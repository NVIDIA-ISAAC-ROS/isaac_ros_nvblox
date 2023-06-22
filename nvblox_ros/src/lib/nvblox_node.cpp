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

#include "nvblox_ros/nvblox_node.hpp"

#include <nvblox/io/mesh_io.h>
#include <nvblox/io/pointcloud_io.h>
#include <nvblox/utils/timing.h>

#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <nvblox_ros_common/qos.hpp>

#include "nvblox_ros/visualization.hpp"

namespace nvblox
{

NvbloxNode::NvbloxNode(
  const ros::NodeOptions & options,
  const std::string & node_name)
: Node(node_name, options), transformer_(this)
{
  // Get parameters first (stuff below depends on parameters)
  getParameters();

  // Set the transformer settings.
  transformer_.set_global_frame(global_frame_);
  transformer_.set_pose_frame(pose_frame_);

  // Initialize the mapper (the interface to the underlying nvblox library)
  // Note: This needs to be called after getParameters()
  // The mapper includes:
  // - Map layers
  // - Integrators
  const std::string mapper_name = "mapper";
  declareMapperParameters(mapper_name, this);
  mapper_ = std::make_shared<Mapper>(
    voxel_size_, MemoryType::kDevice,
    static_projective_layer_type_);
  initializeMapper(mapper_name, mapper_.get(), this);

  // Setup interactions with ROS
  subscribeToTopics();
  setupTimers();
  advertiseTopics();
  advertiseServices();

  // Start the message statistics
  depth_frame_statistics_.Start();
  rgb_frame_statistics_.Start();
  pointcloud_frame_statistics_.Start();

  ros_INFO_STREAM(
     , "Started up nvblox node in frame " <<
      global_frame_ << " and voxel size " <<
      voxel_size_);

  // Set state.
  last_depth_update_time_ = ros::Time(0ul, get_clock()->get_clock_type());
  last_color_update_time_ = ros::Time(0ul, get_clock()->get_clock_type());
  last_lidar_update_time_ = ros::Time(0ul, get_clock()->get_clock_type());
}

void NvbloxNode::getParameters()
{
  ros_INFO_STREAM( , "NvbloxNode::getParameters()");

  const bool is_occupancy =
    declare_parameter<bool>("use_static_occupancy_layer", false);
  if (is_occupancy) {
    static_projective_layer_type_ = ProjectiveLayerType::kOccupancy;
    ros_INFO_STREAM(
       ,
      "static_projective_layer_type: occupancy "
      "(Attention: ESDF and Mesh integration is not yet implemented "
      "for occupancy.)");
  } else {
    static_projective_layer_type_ = ProjectiveLayerType::kTsdf;
    ros_INFO_STREAM(
       ,
      "static_projective_layer_type: TSDF"
      " (for occupancy set the use_static_occupancy_layer parameter)");
  }

  // Declare & initialize the parameters.
  voxel_size_ = declare_parameter<float>("voxel_size", voxel_size_);
  global_frame_ = declare_parameter<std::string>("global_frame", global_frame_);
  pose_frame_ = declare_parameter<std::string>("pose_frame", pose_frame_);
  is_realsense_data_ =
    declare_parameter<bool>("is_realsense_data", is_realsense_data_);
  compute_mesh_ = declare_parameter<bool>("compute_mesh", compute_mesh_);
  compute_esdf_ = declare_parameter<bool>("compute_esdf", compute_esdf_);
  esdf_2d_ = declare_parameter<bool>("esdf_2d", esdf_2d_);
  esdf_distance_slice_ =
    declare_parameter<bool>("esdf_distance_slice", esdf_distance_slice_);
  use_color_ = declare_parameter<bool>("use_color", use_color_);
  use_depth_ = declare_parameter<bool>("use_depth", use_depth_);
  use_lidar_ = declare_parameter<bool>("use_lidar", use_lidar_);
  esdf_slice_height_ =
    declare_parameter<float>("esdf_slice_height", esdf_slice_height_);
  esdf_2d_min_height_ =
    declare_parameter<float>("esdf_2d_min_height", esdf_2d_min_height_);
  esdf_2d_max_height_ =
    declare_parameter<float>("esdf_2d_max_height", esdf_2d_max_height_);
  lidar_width_ = declare_parameter<int>("lidar_width", lidar_width_);
  lidar_height_ = declare_parameter<int>("lidar_height", lidar_height_);
  lidar_vertical_fov_rad_ = declare_parameter<float>(
    "lidar_vertical_fov_rad",
    lidar_vertical_fov_rad_);
  slice_visualization_attachment_frame_id_ =
    declare_parameter<std::string>(
    "slice_visualization_attachment_frame_id",
    slice_visualization_attachment_frame_id_);
  slice_visualization_side_length_ = declare_parameter<float>(
    "slice_visualization_side_length", slice_visualization_side_length_);

  // Update rates
  max_depth_update_hz_ =
    declare_parameter<float>("max_depth_update_hz", max_depth_update_hz_);
  max_color_update_hz_ =
    declare_parameter<float>("max_color_update_hz", max_color_update_hz_);
  max_lidar_update_hz_ =
    declare_parameter<float>("max_lidar_update_hz", max_lidar_update_hz_);
  mesh_update_rate_hz_ =
    declare_parameter<float>("mesh_update_rate_hz", mesh_update_rate_hz_);
  esdf_update_rate_hz_ =
    declare_parameter<float>("esdf_update_rate_hz", esdf_update_rate_hz_);
  occupancy_publication_rate_hz_ = declare_parameter<float>(
    "occupancy_publication_rate_hz", occupancy_publication_rate_hz_);
  max_poll_rate_hz_ =
    declare_parameter<float>("max_poll_rate_hz", max_poll_rate_hz_);

  maximum_sensor_message_queue_length_ =
    declare_parameter<int>(
    "maximum_sensor_message_queue_length",
    maximum_sensor_message_queue_length_);

  // Settings for QoS.
  depth_qos_str_ = declare_parameter<std::string>("depth_qos", depth_qos_str_);
  color_qos_str_ = declare_parameter<std::string>("color_qos", color_qos_str_);

  // Settings for map clearing
  map_clearing_radius_m_ =
    declare_parameter<float>("map_clearing_radius_m", map_clearing_radius_m_);
  map_clearing_frame_id_ = declare_parameter<std::string>(
    "map_clearing_frame_id", map_clearing_frame_id_);
  clear_outside_radius_rate_hz_ = declare_parameter<float>(
    "clear_outside_radius_rate_hz", clear_outside_radius_rate_hz_);
}

void NvbloxNode::subscribeToTopics()
{
  ros_INFO_STREAM( , "NvbloxNode::subscribeToTopics()");

  constexpr int kQueueSize = 10;

  if (!use_depth_ && !use_lidar_) {
    ros_WARN(
       ,
      "Nvblox is running without depth or lidar input, the cost maps and"
      " reconstructions will not update");
  }

  if (use_depth_) {
    // Subscribe to synchronized depth + cam_info topics
    depth_sub_.subscribe(this, "depth/image", parseQosString(depth_qos_str_));
    depth_camera_info_sub_.subscribe(this, "depth/camera_info");

    timesync_depth_.reset(
      new message_filters::Synchronizer<time_policy_t>(
        time_policy_t(kQueueSize), depth_sub_, depth_camera_info_sub_));
    timesync_depth_->registerCallback(
      std::bind(
        &NvbloxNode::depthImageCallback,
        this, std::placeholders::_1,
        std::placeholders::_2));
  }
  if (use_color_) {
    // Subscribe to synchronized color + cam_info topics
    color_sub_.subscribe(this, "color/image", parseQosString(color_qos_str_));
    color_camera_info_sub_.subscribe(this, "color/camera_info");

    timesync_color_.reset(
      new message_filters::Synchronizer<time_policy_t>(
        time_policy_t(kQueueSize), color_sub_, color_camera_info_sub_));
    timesync_color_->registerCallback(
      std::bind(
        &NvbloxNode::colorImageCallback,
        this, std::placeholders::_1,
        std::placeholders::_2));
  }

  if (use_lidar_) {
    // Subscribe to pointclouds.
    pointcloud_sub_ = create_subscription<sensor_msgs::PointCloud2>(
      "pointcloud", kQueueSize,
      std::bind(
        &NvbloxNode::pointcloudCallback, this,
        std::placeholders::_1));
  }

  // Subscribe to transforms.
  transform_sub_ = create_subscription<geometry_msgs::TransformStamped>(
    "transform", kQueueSize,
    std::bind(
      &Transformer::transformCallback, &transformer_,
      std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::PoseStamped>(
    "pose", 10,
    std::bind(
      &Transformer::poseCallback, &transformer_,
      std::placeholders::_1));
}

void NvbloxNode::advertiseTopics()
{
  ros_INFO_STREAM( , "NvbloxNode::advertiseTopics()");

  mesh_publisher_ = create_publisher<nvblox_msgs::Mesh>("~/mesh", 1);
  esdf_pointcloud_publisher_ =
    create_publisher<sensor_msgs::PointCloud2>("~/esdf_pointcloud", 1);
  map_slice_publisher_ =
    create_publisher<nvblox_msgs::DistanceMapSlice>("~/map_slice", 1);
  mesh_marker_publisher_ =
    create_publisher<visualization_msgs::MarkerArray>(
    "~/mesh_marker",
    1);
  slice_bounds_publisher_ = create_publisher<visualization_msgs::Marker>(
    "~/map_slice_bounds", 1);
  occupancy_publisher_ =
    create_publisher<sensor_msgs::PointCloud2>("~/occupancy", 1);
}

void NvbloxNode::advertiseServices()
{
  ros_INFO_STREAM( , "NvbloxNode::advertiseServices()");

  save_ply_service_ = create_service<nvblox_msgs::srv::FilePath>(
    "~/save_ply",
    std::bind(
      &NvbloxNode::savePly, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, group_processing_);
  save_map_service_ = create_service<nvblox_msgs::srv::FilePath>(
    "~/save_map",
    std::bind(
      &NvbloxNode::saveMap, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, group_processing_);
  load_map_service_ = create_service<nvblox_msgs::srv::FilePath>(
    "~/load_map",
    std::bind(
      &NvbloxNode::loadMap, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, group_processing_);
}

void NvbloxNode::setupTimers()
{
  ros_INFO_STREAM( , "NvbloxNode::setupTimers()");
  if (use_depth_) {
    depth_processing_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / max_poll_rate_hz_),
      std::bind(&NvbloxNode::processDepthQueue, this), group_processing_);
  }
  if (use_color_) {
    color_processing_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / max_poll_rate_hz_),
      std::bind(&NvbloxNode::processColorQueue, this), group_processing_);
  }
  if (use_lidar_) {
    pointcloud_processing_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / max_poll_rate_hz_),
      std::bind(&NvbloxNode::processPointcloudQueue, this),
      group_processing_);
  }
  esdf_processing_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / esdf_update_rate_hz_),
    std::bind(&NvbloxNode::processEsdf, this), group_processing_);
  mesh_processing_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / mesh_update_rate_hz_),
    std::bind(&NvbloxNode::processMesh, this), group_processing_);

  if (static_projective_layer_type_ == ProjectiveLayerType::kOccupancy) {
    occupancy_publishing_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / occupancy_publication_rate_hz_),
      std::bind(&NvbloxNode::publishOccupancyPointcloud, this),
      group_processing_);
  }

  if (map_clearing_radius_m_ > 0.0f) {
    clear_outside_radius_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / clear_outside_radius_rate_hz_),
      std::bind(&NvbloxNode::clearMapOutsideOfRadiusOfLastKnownPose, this),
      group_processing_);
  }
}

void NvbloxNode::depthImageCallback(
  const sensor_msgs::ImageConstPtr & depth_img_ptr,
  const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg)
{
  /*
  printMessageArrivalStatistics(
    *depth_img_ptr, "Depth Statistics",
    &depth_frame_statistics_);*/
  pushMessageOntoQueue(
    {depth_img_ptr, camera_info_msg}, &depth_image_queue_,
    &depth_queue_mutex_);
}

void NvbloxNode::colorImageCallback(
  const sensor_msgs::ImageConstPtr & color_image_ptr,
  const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg)
{
  /*
  printMessageArrivalStatistics(
    *color_image_ptr, "Color Statistics",
    &rgb_frame_statistics_);*/
  pushMessageOntoQueue(
    {color_image_ptr, camera_info_msg}, &color_image_queue_,
    &color_queue_mutex_);
}

void NvbloxNode::pointcloudCallback(
  const sensor_msgs::PointCloud2::ConstPtr pointcloud)
{
  /*
  printMessageArrivalStatistics(
    *pointcloud, "Pointcloud Statistics",
    &pointcloud_frame_statistics_);*/
  pushMessageOntoQueue(
    pointcloud, &pointcloud_queue_,
    &pointcloud_queue_mutex_);
}

void NvbloxNode::processDepthQueue()
{
  using ImageInfoMsgPair =
    std::pair<sensor_msgs::ImageConstPtr,
      sensor_msgs::CameraInfo::ConstPtr>;
  auto message_ready = [this](const ImageInfoMsgPair & msg) {
      return this->canTransform(msg.first->header);
    };

  processMessageQueue<ImageInfoMsgPair>(
    &depth_image_queue_,    // NOLINT
    &depth_queue_mutex_,    // NOLINT
    message_ready,          // NOLINT
    std::bind(&NvbloxNode::processDepthImage, this, std::placeholders::_1));

  limitQueueSizeByDeletingOldestMessages(
    maximum_sensor_message_queue_length_,
    "depth", &depth_image_queue_,
    &depth_queue_mutex_);
}

void NvbloxNode::processColorQueue()
{
  using ImageInfoMsgPair =
    std::pair<sensor_msgs::ImageConstPtr,
      sensor_msgs::CameraInfo::ConstPtr>;
  auto message_ready = [this](const ImageInfoMsgPair & msg) {
      return this->canTransform(msg.first->header);
    };

  processMessageQueue<ImageInfoMsgPair>(
    &color_image_queue_,    // NOLINT
    &color_queue_mutex_,    // NOLINT
    message_ready,          // NOLINT
    std::bind(&NvbloxNode::processColorImage, this, std::placeholders::_1));

  limitQueueSizeByDeletingOldestMessages(
    maximum_sensor_message_queue_length_,
    "color", &color_image_queue_,
    &color_queue_mutex_);
}

void NvbloxNode::processPointcloudQueue()
{
  using PointcloudMsg = sensor_msgs::PointCloud2::ConstPtr;
  auto message_ready = [this](const PointcloudMsg & msg) {
      return this->canTransform(msg->header);
    };
  processMessageQueue<PointcloudMsg>(
    &pointcloud_queue_,          // NOLINT
    &pointcloud_queue_mutex_,    // NOLINT
    message_ready,               // NOLINT
    std::bind(
      &NvbloxNode::processLidarPointcloud, this,
      std::placeholders::_1));

  limitQueueSizeByDeletingOldestMessages(
    maximum_sensor_message_queue_length_,
    "pointcloud", &pointcloud_queue_,
    &pointcloud_queue_mutex_);
}

void NvbloxNode::processEsdf()
{
  if (!compute_esdf_) {
    return;
  }
  const ros::Time timestamp = get_clock()->now();
  timing::Timer ros_total_timer("ros/total");
  timing::Timer ros_esdf_timer("ros/esdf");

  timing::Timer esdf_integration_timer("ros/esdf/integrate");
  std::vector<Index3D> updated_blocks;
  if (esdf_2d_) {
    updated_blocks = mapper_->updateEsdfSlice(
      esdf_2d_min_height_, esdf_2d_max_height_, esdf_slice_height_);
  } else {
    updated_blocks = mapper_->updateEsdf();
  }
  esdf_integration_timer.Stop();

  if (updated_blocks.empty()) {
    return;
  }

  timing::Timer esdf_output_timer("ros/esdf/output");

  // If anyone wants a slice
  if (esdf_distance_slice_ &&
    (esdf_pointcloud_publisher_->get_subscription_count() > 0 ||
    map_slice_publisher_->get_subscription_count() > 0))
  {
    // Get the slice as an image
    timing::Timer esdf_slice_compute_timer("ros/esdf/output/compute");
    AxisAlignedBoundingBox aabb;
    Image<float> map_slice_image;
    esdf_slice_converter_.distanceMapSliceImageFromLayer(
      mapper_->esdf_layer(), esdf_slice_height_, &map_slice_image, &aabb);
    esdf_slice_compute_timer.Stop();

    // Slice pointcloud for RVIZ
    if (esdf_pointcloud_publisher_->get_subscription_count() > 0) {
      timing::Timer esdf_output_pointcloud_timer("ros/esdf/output/pointcloud");
      sensor_msgs::PointCloud2 pointcloud_msg;
      esdf_slice_converter_.sliceImageToPointcloud(
        map_slice_image, aabb, esdf_slice_height_,
        mapper_->esdf_layer().voxel_size(), &pointcloud_msg);
      pointcloud_msg.header.frame_id = global_frame_;
      pointcloud_msg.header.stamp = get_clock()->now();
      esdf_pointcloud_publisher_->publish(pointcloud_msg);
    }

    // Also publish the map slice (costmap for nav2).
    if (map_slice_publisher_->get_subscription_count() > 0) {
      timing::Timer esdf_output_human_slice_timer("ros/esdf/output/slice");
      nvblox_msgs::DistanceMapSlice map_slice_msg;
      esdf_slice_converter_.distanceMapSliceImageToMsg(
        map_slice_image, aabb, esdf_slice_height_,
        mapper_->voxel_size_m(), &map_slice_msg);
      map_slice_msg.header.frame_id = global_frame_;
      map_slice_msg.header.stamp = get_clock()->now();
      map_slice_publisher_->publish(map_slice_msg);
    }
  }

  // Also publish the slice bounds (showing esdf max/min 2d height)
  if (slice_bounds_publisher_->get_subscription_count() > 0) {
    // The frame to which the slice limits visualization is attached.
    // We get the transform from the plane-body (PB) frame, to the scene (S).
    Transform T_S_PB;
    if (transformer_.lookupTransformToGlobalFrame(
        slice_visualization_attachment_frame_id_, ros::Time(0),
        &T_S_PB))
    {
      // Get and publish the planes representing the slice bounds in z.
      const visualization_msgs::Marker marker = sliceLimitsToMarker(
        T_S_PB, slice_visualization_side_length_, timestamp, global_frame_,
        esdf_2d_min_height_, esdf_2d_max_height_);
      slice_bounds_publisher_->publish(marker);
    } else {
      constexpr float kTimeBetweenDebugMessages = 1.0;
      ros_INFO_STREAM_THROTTLE(
         , *get_clock(), kTimeBetweenDebugMessages,
        "Tried to publish slice bounds but couldn't look up frame: " <<
          slice_visualization_attachment_frame_id_);
    }
  }
}

void NvbloxNode::processMesh()
{
  if (!compute_mesh_) {
    return;
  }
  const ros::Time timestamp = get_clock()->now();
  timing::Timer ros_total_timer("ros/total");
  timing::Timer ros_mesh_timer("ros/mesh");

  timing::Timer mesh_integration_timer("ros/mesh/integrate_and_color");
  const std::vector<Index3D> mesh_updated_list = mapper_->updateMesh();
  mesh_integration_timer.Stop();

  // In the case that some mesh blocks have been re-added after deletion, remove
  // them from the deleted list.
  for (const Index3D & idx : mesh_updated_list) {
    mesh_blocks_deleted_.erase(idx);
  }
  // Make a list to be published to rviz of blocks to be removed from the viz
  const std::vector<Index3D> mesh_blocks_to_delete(mesh_blocks_deleted_.begin(),
    mesh_blocks_deleted_.end());
  mesh_blocks_deleted_.clear();

  bool should_publish = !mesh_updated_list.empty();

  // Publish the mesh updates.
  timing::Timer mesh_output_timer("ros/mesh/output");
  size_t new_subscriber_count = mesh_publisher_->get_subscription_count();
  if (new_subscriber_count > 0) {
    nvblox_msgs::Mesh mesh_msg;
    // In case we have new subscribers, publish the ENTIRE map once.
    if (new_subscriber_count > mesh_subscriber_count_) {
      ros_INFO( , "Got a new subscriber, sending entire map.");
      conversions::meshMessageFromMeshLayer(mapper_->mesh_layer(), &mesh_msg);
      mesh_msg.clear = true;
      should_publish = true;
    } else {
      conversions::meshMessageFromMeshBlocks(
        mapper_->mesh_layer(),
        mesh_updated_list, &mesh_msg,
        mesh_blocks_to_delete);
    }
    mesh_msg.header.frame_id = global_frame_;
    mesh_msg.header.stamp = timestamp;
    if (should_publish) {
      mesh_publisher_->publish(mesh_msg);
    }
  }
  mesh_subscriber_count_ = new_subscriber_count;

  // optionally publish the markers.
  if (mesh_marker_publisher_->get_subscription_count() > 0) {
    visualization_msgs::MarkerArray marker_msg;
    conversions::markerMessageFromMeshLayer(
      mapper_->mesh_layer(), global_frame_,
      &marker_msg);
    mesh_marker_publisher_->publish(marker_msg);
  }

  mesh_output_timer.Stop();
}

bool NvbloxNode::canTransform(const std_msgs::Header & header)
{
  Transform T_L_C;
  return transformer_.lookupTransformToGlobalFrame(
    header.frame_id,
    header.stamp, &T_L_C);
}

bool NvbloxNode::isUpdateTooFrequent(
  const ros::Time & current_stamp,
  const ros::Time & last_update_stamp,
  float max_update_rate_hz)
{
  if (max_update_rate_hz > 0.0f &&
    (current_stamp - last_update_stamp).seconds() <
    1.0f / max_update_rate_hz)
  {
    return true;
  }
  return false;
}

bool NvbloxNode::processDepthImage(
  const std::pair<sensor_msgs::ImageConstPtr,
  sensor_msgs::CameraInfo::ConstPtr> &
  depth_camera_pair)
{
  timing::Timer ros_depth_timer("ros/depth");
  timing::Timer transform_timer("ros/depth/transform");

  // Message parts
  const sensor_msgs::ImageConstPtr & depth_img_ptr =
    depth_camera_pair.first;
  const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg =
    depth_camera_pair.second;

  // Check that we're not updating more quickly than we should.
  if (isUpdateTooFrequent(
      depth_img_ptr->header.stamp, last_depth_update_time_,
      max_depth_update_hz_))
  {
    return true;
  }
  last_depth_update_time_ = depth_img_ptr->header.stamp;

  // Get the TF for this image.
  Transform T_L_C;
  std::string target_frame = depth_img_ptr->header.frame_id;

  if (!transformer_.lookupTransformToGlobalFrame(
      target_frame, depth_img_ptr->header.stamp, &T_L_C))
  {
    return false;
  }
  transform_timer.Stop();

  timing::Timer conversions_timer("ros/depth/conversions");
  // Convert camera info message to camera object.
  Camera camera = conversions::cameraFromMessage(*camera_info_msg);

  // Convert the depth image.
  if (!conversions::depthImageFromImageMessage(depth_img_ptr, &depth_image_)) {
    ros_ERROR( , "Failed to transform depth image.");
    return false;
  }
  conversions_timer.Stop();

  // Integrate
  timing::Timer integration_timer("ros/depth/integrate");
  mapper_->integrateDepth(depth_image_, T_L_C, camera);
  integration_timer.Stop();
  return true;
}

bool NvbloxNode::processColorImage(
  const std::pair<sensor_msgs::ImageConstPtr,
  sensor_msgs::CameraInfo::ConstPtr> &
  color_camera_pair)
{
  timing::Timer ros_color_timer("ros/color");
  timing::Timer transform_timer("ros/color/transform");

  const sensor_msgs::ImageConstPtr & color_img_ptr =
    color_camera_pair.first;
  const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg =
    color_camera_pair.second;

  // Check that we're not updating more quickly than we should.
  if (isUpdateTooFrequent(
      color_img_ptr->header.stamp, last_color_update_time_,
      max_color_update_hz_))
  {
    return true;
  }
  last_color_update_time_ = color_img_ptr->header.stamp;

  // Get the TF for this image.
  const std::string target_frame = color_img_ptr->header.frame_id;
  Transform T_L_C;

  if (!transformer_.lookupTransformToGlobalFrame(
      target_frame, color_img_ptr->header.stamp, &T_L_C))
  {
    return false;
  }

  transform_timer.Stop();

  timing::Timer color_convert_timer("ros/color/conversion");

  // Convert camera info message to camera object.
  Camera camera = conversions::cameraFromMessage(*camera_info_msg);

  // Convert the color image.
  if (!conversions::colorImageFromImageMessage(color_img_ptr, &color_image_)) {
    ros_ERROR( , "Failed to transform color image.");
    return false;
  }
  color_convert_timer.Stop();

  // Integrate.
  timing::Timer color_integrate_timer("ros/color/integrate");
  mapper_->integrateColor(color_image_, T_L_C, camera);
  color_integrate_timer.Stop();
  return true;
}

bool NvbloxNode::processLidarPointcloud(
  const sensor_msgs::PointCloud2::ConstPtr & pointcloud_ptr)
{
  timing::Timer ros_lidar_timer("ros/lidar");
  timing::Timer transform_timer("ros/lidar/transform");

  // Check that we're not updating more quickly than we should.
  if (isUpdateTooFrequent(
      pointcloud_ptr->header.stamp, last_lidar_update_time_,
      max_lidar_update_hz_))
  {
    return true;
  }
  last_lidar_update_time_ = pointcloud_ptr->header.stamp;

  // Get the TF for this image.
  const std::string target_frame = pointcloud_ptr->header.frame_id;
  Transform T_L_C;

  if (!transformer_.lookupTransformToGlobalFrame(
      target_frame, pointcloud_ptr->header.stamp, &T_L_C))
  {
    return false;
  }

  transform_timer.Stop();

  // LiDAR intrinsics model
  Lidar lidar(lidar_width_, lidar_height_, lidar_vertical_fov_rad_);

  // We check that the pointcloud is consistent with this LiDAR model
  // NOTE(alexmillane): If the check fails we return true which indicates that
  // this pointcloud can be removed from the queue even though it wasn't
  // integrated (because the intrisics model is messed up).
  // NOTE(alexmillane): Note that internally we cache checks, so each LiDAR
  // intrisics model is only tested against a single pointcloud. This is because
  // the check is expensive to perform.
  if (!pointcloud_converter_.checkLidarPointcloud(pointcloud_ptr, lidar)) {
    ros_ERROR_ONCE(
       ,
      "LiDAR intrinsics are inconsistent with the received "
      "pointcloud. Failing integration.");
    return true;
  }

  timing::Timer lidar_conversion_timer("ros/lidar/conversion");
  pointcloud_converter_.depthImageFromPointcloudGPU(
    pointcloud_ptr, lidar,
    &pointcloud_image_);
  lidar_conversion_timer.Stop();

  timing::Timer lidar_integration_timer("ros/lidar/integration");

  mapper_->integrateLidarDepth(pointcloud_image_, T_L_C, lidar);
  lidar_integration_timer.Stop();

  return true;
}

void NvbloxNode::publishOccupancyPointcloud()
{
  timing::Timer ros_total_timer("ros/total");
  timing::Timer esdf_output_timer("ros/occupancy/output");

  if (occupancy_publisher_->get_subscription_count() > 0) {
    sensor_msgs::PointCloud2 pointcloud_msg;
    layer_converter_.pointcloudMsgFromLayer(mapper_->occupancy_layer(), &pointcloud_msg);
    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = get_clock()->now();
    occupancy_publisher_->publish(pointcloud_msg);
  }
}

void NvbloxNode::clearMapOutsideOfRadiusOfLastKnownPose()
{
  if (map_clearing_radius_m_ > 0.0f) {
    timing::Timer("ros/clear_outside_radius");
    Transform T_L_MC;  // MC = map clearing frame
    if (transformer_.lookupTransformToGlobalFrame(
        map_clearing_frame_id_,
        ros::Time(0), &T_L_MC))
    {
      const std::vector<Index3D> blocks_cleared = mapper_->clearOutsideRadius(
        T_L_MC.translation(), map_clearing_radius_m_);
      // We keep track of the deleted blocks for publishing later.
      mesh_blocks_deleted_.insert(blocks_cleared.begin(), blocks_cleared.end());
    } else {
      constexpr float kTimeBetweenDebugMessages = 1.0;
      ros_INFO_STREAM_THROTTLE(
         , *get_clock(), kTimeBetweenDebugMessages,
        "Tried to clear map outside of radius but couldn't look up frame: " <<
          map_clearing_frame_id_);
    }
  }
}

// Helper function for ends with. :)
bool ends_with(const std::string & value, const std::string & ending)
{
  if (ending.size() > value.size()) {
    return false;
  }
  return std::equal(
    ending.crbegin(), ending.crend(), value.crbegin(),
    [](const unsigned char a, const unsigned char b) {
      return std::tolower(a) == std::tolower(b);
    });
}

void NvbloxNode::savePly(
  const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
  std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response)
{
  // If we get a full path, then write to that path.
  bool success = false;
  if (ends_with(request->file_path, ".ply")) {
    success =
      io::outputMeshLayerToPly(mapper_->mesh_layer(), request->file_path);
  } else {
    // If we get a partial path then output a bunch of stuff to a folder.
    io::outputVoxelLayerToPly(
      mapper_->tsdf_layer(),
      request->file_path + "/ros2_tsdf.ply");
    io::outputVoxelLayerToPly(
      mapper_->esdf_layer(),
      request->file_path + "/ros2_esdf.ply");
    success = io::outputMeshLayerToPly(
      mapper_->mesh_layer(),
      request->file_path + "/ros2_mesh.ply");
  }
  if (success) {
    ros_INFO_STREAM(
       ,
      "Output PLY file(s) to " << request->file_path);
    response->success = true;
  } else {
    ros_WARN_STREAM(
       ,
      "Failed to write PLY file(s) to " << request->file_path);
    response->success = false;
  }
}

void NvbloxNode::saveMap(
  const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
  std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response)
{
  std::unique_lock<std::mutex> lock1(depth_queue_mutex_);
  std::unique_lock<std::mutex> lock2(color_queue_mutex_);

  std::string filename = request->file_path;
  if (!ends_with(request->file_path, ".nvblx")) {
    filename += ".nvblx";
  }

  response->success = mapper_->saveMap(filename);
  if (response->success) {
    ros_INFO_STREAM( , "Output map to file to " << filename);
  } else {
    ros_WARN_STREAM( , "Failed to write file to " << filename);
  }
}

void NvbloxNode::loadMap(
  const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
  std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response)
{
  std::unique_lock<std::mutex> lock1(depth_queue_mutex_);
  std::unique_lock<std::mutex> lock2(color_queue_mutex_);

  std::string filename = request->file_path;
  if (!ends_with(request->file_path, ".nvblx")) {
    filename += ".nvblx";
  }

  response->success = mapper_->loadMap(filename);
  if (response->success) {
    ros_INFO_STREAM( , "Loaded map to file from " << filename);
  } else {
    ros_WARN_STREAM(
       ,
      "Failed to load map file from " << filename);
  }
}

}  // namespace nvblox

// Register the node as a component
#include "ros_components/register_node_macro.hpp"
ros_COMPONENTS_REGISTER_NODE(nvblox::NvbloxNode)
