// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "nvblox_ros/fuser_node.hpp"

#include "nvblox_ros/conversions/mesh_conversions.hpp"
#include "nvblox_ros/rosbag_data_loader.hpp"

namespace nvblox
{

FuserNode::FuserNode(const std::string & node_name, std::shared_ptr<CudaStream> cuda_stream)
: Node(node_name), cuda_stream_(cuda_stream)
{
  // Initializing parameters from ROS.
  initializeFuserNodeParams(this, &params_, &parameter_tree_);

  // Initialize layer streamer
  layer_publisher_ = std::make_unique<LayerPublisher>(
    params_.mapping_type,
    params_.layer_visualization_min_tsdf_weight,
    params_.layer_visualization_exclusion_height_m,
    params_.layer_visualization_exclusion_radius_m,
    this);

  RCLCPP_INFO_STREAM(
    get_logger(), "Create nvblox cuda stream with type: "
      << toString(params_.cuda_stream_type.get()));
  cuda_stream_ = CudaStream::createCudaStream(params_.cuda_stream_type.get());

  // Load the selected dataset.
  constexpr int kSeqId = 1;
  constexpr bool kInitFromGflags = false;
  switch (params_.dataset_type) {
    case RosDatasetType::kThreedMatch:
      fuser_ = datasets::threedmatch::createFuser(params_.dataset_path, kSeqId, kInitFromGflags);
      break;
    case RosDatasetType::kRedwood:
      fuser_ = datasets::redwood::createFuser(params_.dataset_path, kInitFromGflags);
      break;
    case RosDatasetType::kReplica:
      fuser_ = datasets::replica::createFuser(params_.dataset_path, kInitFromGflags);
      break;
    case RosDatasetType::kRosbag:
      fuser_ = datasets::ros::createFuser(
        params_.dataset_path, params_.depth_topic,
        params_.depth_camera_info_topic, params_.color_topic,
        params_.color_camera_info_topic, params_.global_frame,
        params_.tf_lead_time_s, cuda_stream_);
      break;
  }

  // Check if loading worked.
  if (!fuser_) {
    RCLCPP_FATAL_STREAM(
      get_logger(), "Creation of " << toString(params_.dataset_type)
                                   << " fuser failed with dataset path: "
                                   << params_.dataset_path.get());
    exit(1);
  }

  // Check if a valid mapping typ was selected.
  if (params_.mapping_type != MappingType::kStaticTsdf) {
    RCLCPP_FATAL_STREAM(
      get_logger(), "Currently the only supported mapping type for the "
      "fuser node is static tsdf.");
    exit(1);
  }

  // Create the mapper.
  fuser_->setMultiMapper(
    std::make_shared<MultiMapper>(
      params_.voxel_size, params_.mapping_type,
      params_.esdf_mode, MemoryType::kDevice,
      cuda_stream_));
  // Set the mapper parameters.
  const std::string mapper_name = "static_mapper";
  declareMapperParameters(mapper_name, this);
  MapperParams mapper_params = getMapperParamsFromROS(mapper_name, this);
  fuser_->multi_mapper()->setMapperParams(mapper_params);

  // Get direct handles to the underlying mapper.
  mapper_ = fuser_->static_mapper();

  // Advertise topics.
  color_frame_publisher_ = create_publisher<sensor_msgs::msg::Image>("~/color_frame", 1);
  depth_frame_publisher_ = create_publisher<sensor_msgs::msg::Image>("~/depth_frame", 1);
  esdf_pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/esdf_pointcloud", 1);
  mesh_publisher_ = create_publisher<nvblox_msgs::msg::Mesh>("~/mesh", 1);
  back_projected_depth_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/back_projected_depth", 1);

  // Initialize tf broadcaster.
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Print parameters to console.
  CHECK(parameter_tree_.children().has_value());
  parameter_tree_.children().value().push_back(mapper_->getParameterTree());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "fuser parameters:\n"
      << nvblox::parameters::parameterTreeToString(parameter_tree_));

  // Let the world know that this was a full success.
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Created " << toString(params_.dataset_type)
               << " fuser with dataset path: " << params_.dataset_path.get());
  // If we integrate frames on key input, we need to set up terminal to
  // detect entered keys.
  // if (params_.update_on_key) {
  initTerminal();
  RCLCPP_INFO(
    get_logger(), "\n\n"
    "----------------------------------------------\n"
    "         Set up reading from terminal         \n"
    "  Use the SPACE KEY to integrate a new frame  \n"
    "----------------------------------------------\n");
  // }
}

bool FuserNode::update()
{
  // If update_on_key is true, we fuse a new frame if a space key is
  // detected. Otherwise we just fuse the next frame.
  if (params_.update_on_key) {
    return updateOnKey();
  } else {
    // return fuseNextFrame();
    return updateIfNotPaused();
  }
}

bool FuserNode::updateIfNotPaused()
{
  // Update state if requested
  char c;
  if (readFromTerminal(&c)) {
    if (c == ' ') {
      switch (continuous_mode_state_) {
        case ContinuousModeState::kRunning:
          continuous_mode_state_ = ContinuousModeState::kPaused;
          RCLCPP_INFO(get_logger(), "Pausing.");
          break;
        case ContinuousModeState::kPaused:
          continuous_mode_state_ = ContinuousModeState::kRunning;
          RCLCPP_INFO(get_logger(), "Running.");
          break;
        default:
          RCLCPP_FATAL(get_logger(), "Unhandled state.");
          break;
      }
    } else {
      RCLCPP_INFO(get_logger(), "Key does nothing. Only space does something in continuous mode.");
    }
  }

  // Fuse more frames if we're not paused
  if (continuous_mode_state_ == ContinuousModeState::kRunning) {
    return fuseNextFrame();
  }
  return true;
}

bool FuserNode::updateOnKey()
{
  char c;
  if (readFromTerminal(&c)) {
    // Do an action depending on the entered key.
    switch (c) {
      case ' ':
        return fuseNextFrame();
      case 'p':
        printStatistics();
        break;
      case 'f':
        RCLCPP_INFO_STREAM(get_logger(), nvblox::art::PrintNvbloxFrog());
        break;
      default:
        RCLCPP_ERROR_STREAM(get_logger(), "Invalid key detected: " << c);
        break;
    }
  }
  return true;
}

bool FuserNode::fuseNextFrame()
{
  // Check whether we should try integrating the next frame.
  // If number_of_frames_to_integrate is negative we integrate all available
  // frames.
  const bool fuse_next_frame = params_.number_of_frames_to_integrate < 0 ||
    current_frame_number_ < params_.number_of_frames_to_integrate;
  if (!fuse_next_frame) {
    RCLCPP_INFO(get_logger(), "Finished integrating frames.");
    printStatistics();
    return false;
  }

  // Fuse the next frame.
  datasets::DataLoadResult fuse_result = fuser_->integrateFrame(current_frame_number_++);
  if (fuse_result == datasets::DataLoadResult::kBadFrame) {
    // Bad frame but keep going
    return true;
  } else if (fuse_result == datasets::DataLoadResult::kNoMoreData) {
    RCLCPP_INFO(get_logger(), "Finished integrating frames.");
    printStatistics();
    return false;
  }

  // Gather data for publishing results of fuser step.
  constexpr char kCameraFrameID[] = "camera_link";
  const rclcpp::Time timestamp = get_clock()->now();
  const std::shared_ptr<const DepthImage> depth_frame = fuser_->getDepthFrame();
  const std::shared_ptr<const ColorImage> color_frame = fuser_->getColorFrame();
  const std::shared_ptr<const Camera> depth_camera = fuser_->getDepthCamera();
  const std::shared_ptr<const Transform> depth_T_L_C = fuser_->getDepthCameraPose();

  // Publish the current pose of the camera to tf.
  geometry_msgs::msg::TransformStamped transform_stamped_msg;
  conversions::transformToTransformStampedMsg(
    *depth_T_L_C, params_.global_frame, kCameraFrameID,
    timestamp, &transform_stamped_msg);
  tf_broadcaster_->sendTransform(transform_stamped_msg);

  // Publish the color frame.
  if (color_frame_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::Image color_img_msg;
    conversions::imageMessageFromColorImage(
      *color_frame, kCameraFrameID, &color_img_msg,
      *cuda_stream_);
    color_frame_publisher_->publish(color_img_msg);
  }

  // Publish the depth frame.
  if (depth_frame_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::Image depth_img_msg;
    conversions::imageMessageFromDepthImage(
      *depth_frame, kCameraFrameID, &depth_img_msg,
      *cuda_stream_);
    depth_frame_publisher_->publish(depth_img_msg);
  }

  // Publish the esdf slice pointcloud.
  if (esdf_pointcloud_publisher_->get_subscription_count() > 0) {
    // Get the slice image from the esdf layer.
    AxisAlignedBoundingBox aabb;
    Image<float> map_slice_image(MemoryType::kDevice);
    esdf_slice_converter_.sliceLayerToDistanceImage(
      mapper_->esdf_layer(), mapper_->esdf_integrator().esdf_slice_height(),
      params_.distance_map_unknown_value_optimistic, &map_slice_image, &aabb);
    // Create and send the message.
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    esdf_slice_converter_.pointcloudMsgFromSliceImage(
      map_slice_image, aabb, mapper_->esdf_integrator().esdf_slice_height(),
      mapper_->esdf_layer().voxel_size(),
      params_.distance_map_unknown_value_optimistic, &pointcloud_msg);
    pointcloud_msg.header.frame_id = params_.global_frame;
    pointcloud_msg.header.stamp = timestamp;
    esdf_pointcloud_publisher_->publish(pointcloud_msg);
  }

  // Publish the mesh.
  if (mesh_publisher_->get_subscription_count() > 0) {
    std::shared_ptr<SerializedColorMeshLayer> serialized_mesh =
      fuser_->getSerializedColorMesh();
    nvblox_msgs::msg::Mesh mesh_msg;
    bool constexpr kResendFullMesh = false;
    conversions::meshMessageFromSerializedMesh(
      serialized_mesh, timestamp, params_.global_frame,
      mapper_->color_mesh_layer().block_size(), kResendFullMesh,
      &mesh_msg);
    mesh_publisher_->publish(mesh_msg);
  }

  // Publish the back projected pointcloud.
  if (back_projected_depth_publisher_->get_subscription_count() > 0) {
    // Get the pointcloud from the depth image.
    image_back_projector_.backProjectOnGPU(
      *depth_frame, *depth_camera, &pointcloud_C_device_,
      params_.max_back_projection_distance);
    transformPointcloudOnGPU(*depth_T_L_C, pointcloud_C_device_, &pointcloud_L_device_);
    // Create and send the message.
    sensor_msgs::msg::PointCloud2 back_projected_depth_msg;
    pointcloud_converter_.pointcloudMsgFromPointcloud(
      pointcloud_L_device_,
      &back_projected_depth_msg);
    back_projected_depth_msg.header.frame_id = params_.global_frame;
    back_projected_depth_msg.header.stamp = timestamp;
    back_projected_depth_publisher_->publish(back_projected_depth_msg);
  }

  // Publish layers
  layer_publisher_->serializeAndpublishSubscribedLayers(
    *depth_T_L_C, params_.global_frame, timestamp, params_.layer_streamer_bandwidth_limit_mbps,
    mapper_, nullptr,
    get_logger());

  return true;
}

void FuserNode::printStatistics()
{
  RCLCPP_INFO_STREAM(get_logger(), "Timing statistics: \n" << nvblox::timing::Timing::Print());
  RCLCPP_INFO_STREAM(get_logger(), "Rates statistics: \n" << nvblox::timing::Rates::Print());
}

}  // namespace nvblox
