// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "nvblox_ros/nvblox_human_node.hpp"

#include <nvblox/io/csv.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <nvblox_ros_common/qos.hpp>


namespace nvblox
{

NvbloxHumanNode::NvbloxHumanNode(const rclcpp::NodeOptions & options)
: NvbloxNode(options, "nvblox_human_node"),
  human_pointcloud_C_device_(MemoryType::kDevice),
  human_pointcloud_L_device_(MemoryType::kDevice)
{
  RCLCPP_INFO_STREAM(get_logger(), "NvbloxHumanNode::NvbloxHumanNode()");

  // Get parameters specific to the human node.
  getParameters();

  // Initialize the MultiMapper and overwrite the base-class node's Mapper.
  initializeMultiMapper();

  // Subscribe to topics
  // NOTE(alexmillane): This function modifies to base class subscriptions to
  // add synchronization with segmentation masks.
  subscribeToTopics();

  // Add additional timers and publish more topics
  setupTimers();
  advertiseTopics();
}

void NvbloxHumanNode::getParameters()
{
  human_occupancy_decay_rate_hz_ = declare_parameter<float>(
    "human_occupancy_decay_rate_hz", human_occupancy_decay_rate_hz_);
  human_esdf_update_rate_hz_ = declare_parameter<float>(
    "human_esdf_update_rate_hz", human_esdf_update_rate_hz_);
}

void NvbloxHumanNode::initializeMultiMapper()
{
  // Initialize the multi mapper. Composed of:
  // - masked occupancy mapper for humans
  // - unmasked mapper for static objects (with configurable projective layer
  //   type)
  constexpr ProjectiveLayerType kDynamicLayerType =
    ProjectiveLayerType::kOccupancy;
  multi_mapper_ = std::make_shared<MultiMapper>(
    voxel_size_, MemoryType::kDevice, kDynamicLayerType,
    static_projective_layer_type_);

  // Over-write the base-class node's mapper with the unmasked mapper of
  // the multi mapper. We also have to initialize it from ROS 2 params by
  // calling initializeMapper() (again) (it its also called in the base
  // constructor, on the now-deleted Mapper).
  mapper_ = multi_mapper_.get()->unmasked_mapper();
  initializeMapper("mapper", mapper_.get(), this);
  // Set to an invalid depth to ignore human pixels in the unmasked mapper
  // during integration.
  multi_mapper_->setDepthUnmaskedImageInvalidPixel(-1.f);

  // Initialize the human mapper (masked mapper of the multi mapper)
  const std::string mapper_name = "human_mapper";
  human_mapper_ = multi_mapper_.get()->masked_mapper();
  // Human mapper params have not been declared yet
  declareMapperParameters(mapper_name, this);
  initializeMapper(mapper_name, human_mapper_.get(), this);
  // Set to a distance bigger than the max. integration distance to not include
  // non human pixels on the human mapper, but clear along the projection.
  // TODO(remosteiner): Think of a better way to do this.
  // Currently this leads to blocks being allocated even behind solid obstacles.
  multi_mapper_->setDepthMaskedImageInvalidPixel(
    human_mapper_->occupancy_integrator().max_integration_distance_m() * 2.f);
}

void NvbloxHumanNode::subscribeToTopics()
{
  RCLCPP_INFO_STREAM(get_logger(), "NvbloxHumanNode::subscribeToTopics()");

  // Increased queue size compared to the NvbloxNode,
  // because of bigger delay comming from segmentation.
  constexpr int kQueueSize = 40;

  // Unsubscribe from base-class synchronized topics.
  // We redo synchronization below.
  NvbloxNode::timesync_depth_.reset();
  NvbloxNode::timesync_color_.reset();

  // Subscribe to segmentation masks
  segmentation_mask_sub_.subscribe(
    this, "mask/image",
    parseQosString(color_qos_str_));
  segmentation_camera_info_sub_.subscribe(this, "mask/camera_info");

  if (use_depth_) {
    // Unsubscribe from the depth topic in nvblox_node
    timesync_depth_.reset();
    // Subscribe to depth + mask + cam_infos
    timesync_depth_mask_ = std::make_shared<message_filters::Synchronizer<mask_time_policy_t>>(
      mask_time_policy_t(kQueueSize), depth_sub_, depth_camera_info_sub_,
      segmentation_mask_sub_, segmentation_camera_info_sub_);
    timesync_depth_mask_->registerCallback(
      std::bind(
        &NvbloxHumanNode::depthPlusMaskImageCallback, this,
        std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4));
  }

  if (use_color_) {
    // Unsubscribe from the color topic in nvblox_node
    timesync_color_.reset();
    // Subscribe to color + mask + cam_infos
    timesync_color_mask_ = std::make_shared<message_filters::Synchronizer<mask_time_policy_t>>(
      mask_time_policy_t(kQueueSize), color_sub_, color_camera_info_sub_,
      segmentation_mask_sub_, segmentation_camera_info_sub_);
    timesync_color_mask_->registerCallback(
      std::bind(
        &NvbloxHumanNode::colorPlusMaskImageCallback, this,
        std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4));
  }
}

void NvbloxHumanNode::advertiseTopics()
{
  // Add some stuff
  human_pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/human_pointcloud", 1);
  human_voxels_publisher_ =
    create_publisher<visualization_msgs::msg::Marker>("~/human_voxels", 1);
  human_occupancy_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/human_occupancy", 1);
  human_esdf_pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/human_esdf_pointcloud",
    1);
  combined_esdf_pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/combined_esdf_pointcloud", 1);
  human_map_slice_publisher_ =
    create_publisher<nvblox_msgs::msg::DistanceMapSlice>(
    "~/human_map_slice",
    1);
  combined_map_slice_publisher_ =
    create_publisher<nvblox_msgs::msg::DistanceMapSlice>(
    "~/combined_map_slice", 1);
  depth_frame_overlay_publisher_ =
    create_publisher<sensor_msgs::msg::Image>("~/depth_frame_overlay", 1);
  color_frame_overlay_publisher_ =
    create_publisher<sensor_msgs::msg::Image>("~/color_frame_overlay", 1);
}

void NvbloxHumanNode::setupTimers()
{
  human_occupancy_decay_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / human_occupancy_decay_rate_hz_),
    std::bind(&NvbloxHumanNode::decayHumanOccupancy, this),
    group_processing_);
  human_esdf_processing_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / human_esdf_update_rate_hz_),
    std::bind(&NvbloxHumanNode::processHumanEsdf, this), group_processing_);
}

void NvbloxHumanNode::depthPlusMaskImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_img_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & mask_img_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & mask_camera_info_msg)
{
  printMessageArrivalStatistics(
    *depth_img_ptr, "Depth plus Mask Statistics",
    &depth_frame_statistics_);
  pushMessageOntoQueue<ImageSegmentationMaskMsgTuple>(
    std::make_tuple(
      depth_img_ptr, camera_info_msg, mask_img_ptr,
      mask_camera_info_msg),
    &depth_mask_image_queue_, &depth_mask_queue_mutex_);
}

void NvbloxHumanNode::colorPlusMaskImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & color_img_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & mask_img_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & mask_camera_info_msg)
{
  printMessageArrivalStatistics(
    *color_img_ptr, "Color plus Mask Statistics",
    &rgb_frame_statistics_);
  pushMessageOntoQueue<ImageSegmentationMaskMsgTuple>(
    std::make_tuple(
      color_img_ptr, camera_info_msg, mask_img_ptr,
      mask_camera_info_msg),
    &color_mask_image_queue_, &color_mask_queue_mutex_);
}

void NvbloxHumanNode::processDepthQueue()
{
  auto message_ready = [this](const ImageSegmentationMaskMsgTuple & msg) {
      return this->canTransform(std::get<0>(msg)->header) &&
             this->canTransform(std::get<2>(msg)->header);
    };
  processMessageQueue<ImageSegmentationMaskMsgTuple>(
    &depth_mask_image_queue_,    // NOLINT
    &depth_mask_queue_mutex_,    // NOLINT
    message_ready,               // NOLINT
    std::bind(
      &NvbloxHumanNode::processDepthImage, this,
      std::placeholders::_1));

  limitQueueSizeByDeletingOldestMessages(
    maximum_sensor_message_queue_length_,
    "depth_mask", &depth_mask_image_queue_,
    &depth_mask_queue_mutex_);
}

void NvbloxHumanNode::processColorQueue()
{
  auto message_ready = [this](const ImageSegmentationMaskMsgTuple & msg) {
      return this->canTransform(std::get<0>(msg)->header) &&
             this->canTransform(std::get<2>(msg)->header);
    };
  processMessageQueue<ImageSegmentationMaskMsgTuple>(
    &color_mask_image_queue_,    // NOLINT
    &color_mask_queue_mutex_,    // NOLINT
    message_ready,               // NOLINT
    std::bind(
      &NvbloxHumanNode::processColorImage, this,
      std::placeholders::_1));

  limitQueueSizeByDeletingOldestMessages(
    maximum_sensor_message_queue_length_,
    "color_mask", &color_mask_image_queue_,
    &color_mask_queue_mutex_);
}

bool NvbloxHumanNode::processDepthImage(
  const ImageSegmentationMaskMsgTuple & depth_mask_msg)
{
  timing::Timer ros_total_timer("ros/total");
  timing::Timer ros_depth_timer("ros/depth");
  timing::Timer transform_timer("ros/depth/transform");

  // Message parts
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_img_ptr =
    std::get<0>(depth_mask_msg);
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & depth_camera_info_msg =
    std::get<1>(depth_mask_msg);
  const sensor_msgs::msg::Image::ConstSharedPtr & mask_img_ptr =
    std::get<2>(depth_mask_msg);
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & mask_camera_info_msg =
    std::get<3>(depth_mask_msg);

  // Check that we're not updating more quickly than we should.
  if (isUpdateTooFrequent(
      depth_img_ptr->header.stamp, last_depth_update_time_,
      max_depth_update_hz_))
  {
    return true;
  }
  last_depth_update_time_ = depth_img_ptr->header.stamp;

  // Get the TF for BOTH images.
  const std::string depth_img_frame = depth_img_ptr->header.frame_id;
  if (!transformer_.lookupTransformToGlobalFrame(
      depth_img_frame, depth_img_ptr->header.stamp, &T_L_C_depth_))
  {
    return false;
  }
  Transform T_L_C_mask;
  const std::string mask_img_frame = mask_img_ptr->header.frame_id;
  if (!transformer_.lookupTransformToGlobalFrame(
      mask_img_frame, mask_img_ptr->header.stamp, &T_L_C_mask))
  {
    return false;
  }
  Transform T_CM_CD = T_L_C_mask.inverse() * T_L_C_depth_;
  if (is_realsense_data_) {
    // There is an unresolved issue with the ROS realsense wrapper.
    // Until it is fixed, the below inverse needs to be applied.
    // https://github.com/IntelRealSense/realsense-ros/issues/2500
    T_CM_CD = T_CM_CD.inverse();
  }
  transform_timer.Stop();

  timing::Timer conversions_timer("ros/depth/conversions");
  // Convert camera info message to camera object.
  depth_camera_ = conversions::cameraFromMessage(*depth_camera_info_msg);
  const Camera mask_camera =
    conversions::cameraFromMessage(*mask_camera_info_msg);

  // Convert the depth image.
  if (!conversions::depthImageFromImageMessage(depth_img_ptr, &depth_image_) ||
    !conversions::monoImageFromImageMessage(mask_img_ptr, &mask_image_))
  {
    RCLCPP_ERROR(get_logger(), "Failed to transform depth or mask image.");
    return false;
  }
  conversions_timer.Stop();

  // Integrate
  timing::Timer integration_timer("ros/depth/integrate");
  multi_mapper_->integrateDepth(
    depth_image_, mask_image_, T_L_C_depth_,
    T_CM_CD, depth_camera_, mask_camera);
  integration_timer.Stop();

  timing::Timer overlay_timer("ros/depth/output/human_overlay");
  if (depth_frame_overlay_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::Image img_msg;
    const ColorImage & depth_overlay =
      multi_mapper_->getLastDepthFrameMaskOverlay();
    conversions::imageMessageFromColorImage(
      depth_overlay, depth_img_frame,
      &img_msg);
    depth_frame_overlay_publisher_->publish(img_msg);
  }

  return true;
}

bool NvbloxHumanNode::processColorImage(
  const ImageSegmentationMaskMsgTuple & color_mask_msg)
{
  timing::Timer ros_total_timer("ros/total");
  timing::Timer ros_color_timer("ros/color");
  timing::Timer transform_timer("ros/color/transform");

  // Message parts
  const sensor_msgs::msg::Image::ConstSharedPtr & color_img_ptr =
    std::get<0>(color_mask_msg);
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg =
    std::get<1>(color_mask_msg);
  const sensor_msgs::msg::Image::ConstSharedPtr & mask_img_ptr =
    std::get<2>(color_mask_msg);
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & mask_camera_info_msg =
    std::get<3>(color_mask_msg);

  // Check that we're not updating more quickly than we should.
  if (isUpdateTooFrequent(
      color_img_ptr->header.stamp, last_color_update_time_,
      max_color_update_hz_))
  {
    return true;
  }
  last_color_update_time_ = color_img_ptr->header.stamp;

  // Get the TF for BOTH images.
  Transform T_L_C;
  const std::string color_img_frame = color_img_ptr->header.frame_id;
  if (!transformer_.lookupTransformToGlobalFrame(
      color_img_frame, color_img_ptr->header.stamp, &T_L_C))
  {
    return false;
  }
  Transform T_L_C_mask;
  const std::string mask_img_frame = mask_img_ptr->header.frame_id;
  if (!transformer_.lookupTransformToGlobalFrame(
      mask_img_frame, mask_img_ptr->header.stamp, &T_L_C_mask))
  {
    return false;
  }
  transform_timer.Stop();

  timing::Timer conversions_timer("ros/color/conversions");
  // Convert camera info message to camera object.
  const Camera color_camera = conversions::cameraFromMessage(*camera_info_msg);
  const Camera mask_camera =
    conversions::cameraFromMessage(*mask_camera_info_msg);
  if (!camerasAreEquivalent(color_camera, mask_camera, T_L_C, T_L_C_mask)) {
    RCLCPP_ERROR(
      get_logger(),
      "Color and mask image are not coming from the same camera or frame.");
    return false;
  }

  // Convert the color image.
  if (!conversions::colorImageFromImageMessage(color_img_ptr, &color_image_) ||
    !conversions::monoImageFromImageMessage(mask_img_ptr, &mask_image_))
  {
    RCLCPP_ERROR(get_logger(), "Failed to transform color or mask image.");
    return false;
  }
  conversions_timer.Stop();

  // Integrate
  timing::Timer integration_timer("ros/color/integrate");
  multi_mapper_->integrateColor(color_image_, mask_image_, T_L_C, color_camera);
  integration_timer.Stop();

  timing::Timer overlay_timer("ros/color/output/human_overlay");
  if (color_frame_overlay_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::Image img_msg;
    const ColorImage & color_overlay =
      multi_mapper_->getLastColorFrameMaskOverlay();
    conversions::imageMessageFromColorImage(
      color_overlay, color_img_frame,
      &img_msg);
    color_frame_overlay_publisher_->publish(img_msg);
  }

  return true;
}

void NvbloxHumanNode::processHumanEsdf()
{
  timing::Timer ros_total_timer("ros/total");
  timing::Timer ros_human_total_timer("ros/humans");

  if (last_depth_update_time_.seconds() <= 0.f) {
    return;  // no data yet.
  }
  publishHumanDebugOutput();

  // Process the human esdf layer.
  timing::Timer esdf_integration_timer("ros/humans/esdf/integrate");
  std::vector<Index3D> updated_blocks;
  if (esdf_2d_) {
    updated_blocks = human_mapper_->updateEsdfSlice(
      esdf_2d_min_height_, esdf_2d_max_height_, esdf_slice_height_);
  } else {
    updated_blocks = human_mapper_->updateEsdf();
  }
  esdf_integration_timer.Stop();

  if (updated_blocks.empty()) {
    return;
  }

  timing::Timer esdf_output_timer("ros/humans/esdf/output");

  // Check if anyone wants any human slice
  if (esdf_distance_slice_ &&
    (human_esdf_pointcloud_publisher_->get_subscription_count() > 0) ||
    (human_map_slice_publisher_->get_subscription_count() > 0))
  {
    // Get the slice as an image
    timing::Timer esdf_slice_compute_timer("ros/humans/esdf/output/compute");
    AxisAlignedBoundingBox aabb;
    Image<float> map_slice_image;
    esdf_slice_converter_.distanceMapSliceImageFromLayer(
      human_mapper_->esdf_layer(), esdf_slice_height_, &map_slice_image, &aabb);
    esdf_slice_compute_timer.Stop();

    // Human slice pointcloud (for visualization)
    if (human_esdf_pointcloud_publisher_->get_subscription_count() > 0) {
      timing::Timer esdf_output_human_pointcloud_timer("ros/humans/esdf/output/pointcloud");
      sensor_msgs::msg::PointCloud2 pointcloud_msg;
      esdf_slice_converter_.sliceImageToPointcloud(
        map_slice_image, aabb, esdf_slice_height_,
        human_mapper_->esdf_layer().voxel_size(), &pointcloud_msg);
      pointcloud_msg.header.frame_id = global_frame_;
      pointcloud_msg.header.stamp = get_clock()->now();
      human_esdf_pointcloud_publisher_->publish(pointcloud_msg);
    }

    // Human slice (for navigation)
    if (human_map_slice_publisher_->get_subscription_count() > 0) {
      timing::Timer esdf_output_human_slice_timer("ros/humans/esdf/output/slice");
      nvblox_msgs::msg::DistanceMapSlice map_slice_msg;
      esdf_slice_converter_.distanceMapSliceImageToMsg(
        map_slice_image, aabb, esdf_slice_height_,
        human_mapper_->voxel_size_m(), &map_slice_msg);
      map_slice_msg.header.frame_id = global_frame_;
      map_slice_msg.header.stamp = get_clock()->now();
      human_map_slice_publisher_->publish(map_slice_msg);
    }
  }

  // Check if anyone wants any human+statics slice
  if (esdf_distance_slice_ &&
    (combined_esdf_pointcloud_publisher_->get_subscription_count() >
    0) ||
    (combined_map_slice_publisher_->get_subscription_count() > 0))
  {
    // Combined slice
    timing::Timer esdf_slice_compute_timer("ros/humans/esdf/output/combined/compute");
    Image<float> combined_slice_image;
    AxisAlignedBoundingBox combined_aabb;
    esdf_slice_converter_.distanceMapSliceFromLayers(
      mapper_->esdf_layer(), human_mapper_->esdf_layer(), esdf_slice_height_,
      &combined_slice_image, &combined_aabb);
    esdf_slice_compute_timer.Stop();

    // Human+Static slice pointcloud (for visualization)
    if (combined_esdf_pointcloud_publisher_->get_subscription_count() > 0) {
      timing::Timer esdf_output_human_pointcloud_timer(
        "ros/humans/esdf/output/combined/pointcloud");
      sensor_msgs::msg::PointCloud2 pointcloud_msg;
      esdf_slice_converter_.sliceImageToPointcloud(
        combined_slice_image, combined_aabb, esdf_slice_height_,
        human_mapper_->esdf_layer().voxel_size(), &pointcloud_msg);
      pointcloud_msg.header.frame_id = global_frame_;
      pointcloud_msg.header.stamp = get_clock()->now();
      combined_esdf_pointcloud_publisher_->publish(pointcloud_msg);
    }

    // Human+Static slice (for navigation)
    if (combined_map_slice_publisher_->get_subscription_count() > 0) {
      timing::Timer esdf_output_human_slice_timer("ros/humans/esdf/output/combined/slice");
      nvblox_msgs::msg::DistanceMapSlice map_slice_msg;
      esdf_slice_converter_.distanceMapSliceImageToMsg(
        combined_slice_image, combined_aabb, esdf_slice_height_,
        human_mapper_->voxel_size_m(), &map_slice_msg);
      map_slice_msg.header.frame_id = global_frame_;
      map_slice_msg.header.stamp = get_clock()->now();
      human_map_slice_publisher_->publish(map_slice_msg);
    }
  }
  esdf_output_timer.Stop();
}

void NvbloxHumanNode::decayHumanOccupancy() {human_mapper_->decayOccupancy();}

void NvbloxHumanNode::publishHumanDebugOutput()
{
  timing::Timer ros_human_debug_timer("ros/humans/output/debug");

  // Get a human pointcloud
  if (human_pointcloud_publisher_->get_subscription_count() +
    human_voxels_publisher_->get_subscription_count() >
    0)
  {
    // Grab the human only image.
    const DepthImage & depth_image_only_humans =
      multi_mapper_->getLastDepthFrameMasked();
    // Back project
    image_back_projector_.backProjectOnGPU(
      depth_image_only_humans, depth_camera_, &human_pointcloud_C_device_,
      human_mapper_->occupancy_integrator().max_integration_distance_m());
    transformPointcloudOnGPU(
      T_L_C_depth_, human_pointcloud_C_device_,
      &human_pointcloud_L_device_);
  }

  // Publish the human pointcloud
  if (human_pointcloud_publisher_->get_subscription_count() > 0) {
    // Back-project human depth image to pointcloud and publish.
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pointcloud_converter_.pointcloudMsgFromPointcloud(
      human_pointcloud_L_device_,
      &pointcloud_msg);
    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = get_clock()->now();
    human_pointcloud_publisher_->publish(pointcloud_msg);
  }

  // Publish human voxels
  if (human_voxels_publisher_->get_subscription_count() > 0) {
    // Human voxels from points (in the layer frame)
    image_back_projector_.pointcloudToVoxelCentersOnGPU(
      human_pointcloud_L_device_, voxel_size_,
      &human_voxel_centers_L_device_);
    // Publish
    visualization_msgs::msg::Marker marker_msg;
    pointcloud_converter_.pointsToCubesMarkerMsg(
      human_voxel_centers_L_device_.points().toVector(), voxel_size_,
      Color::Red(), &marker_msg);
    marker_msg.header.frame_id = global_frame_;
    marker_msg.header.stamp = get_clock()->now();
    human_voxels_publisher_->publish(marker_msg);
  }

  // Publish the human occupancy layer
  if (human_occupancy_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    layer_converter_.pointcloudMsgFromLayer(
      human_mapper_->occupancy_layer(),
      &pointcloud_msg);
    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = get_clock()->now();
    human_occupancy_publisher_->publish(pointcloud_msg);
  }
}

}  // namespace nvblox

// Register the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvblox::NvbloxHumanNode)
