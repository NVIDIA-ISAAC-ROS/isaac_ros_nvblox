// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
  const rclcpp::NodeOptions & options,
  const std::string & node_name)
: Node(node_name, options), transformer_(this)
{
  // Get parameters first (stuff below depends on parameters)
  getParameters();

  // Set the transformer settings.
  transformer_.set_global_frame(global_frame_);
  transformer_.set_pose_frame(pose_frame_);

  // Create callback groups, which allows processing to go in parallel with the
  // subscriptions.
  group_processing_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Initialize the MultiMapper with the underlying dynamic/static mappers.
  initializeMultiMapper();

  // Setup interactions with ROS
  subscribeToTopics();
  setupTimers();
  advertiseTopics();
  advertiseServices();

  // Start the message statistics
  depth_frame_statistics_.Start();
  rgb_frame_statistics_.Start();
  pointcloud_frame_statistics_.Start();

  RCLCPP_INFO_STREAM(
    get_logger(), "Started up nvblox node in frame "
      << global_frame_ << " and voxel size "
      << voxel_size_);

  // Check if a valid mapping typ was selected
  RCLCPP_INFO_STREAM(get_logger(), "Mapping type: " << toString(mapping_type_));
  if (get_name() == "nvblox_node") {
    if (isHumanMapping(mapping_type_)) {
      RCLCPP_FATAL_STREAM(
        get_logger(),
        "Invalid option. The basic nvblox node can not do human mapping. "
        "Use the nvblox human node for that. Exiting nvblox.");
      exit(1);
    }
  }

  // Set state.
  last_depth_update_time_ = rclcpp::Time(0ul, get_clock()->get_clock_type());
  last_color_update_time_ = rclcpp::Time(0ul, get_clock()->get_clock_type());
  last_lidar_update_time_ = rclcpp::Time(0ul, get_clock()->get_clock_type());
}

void NvbloxNode::getParameters()
{
  // Declare & initialize the parameters.
  RCLCPP_INFO_STREAM(get_logger(), "NvbloxNode::getParameters()");

  // Node params
  global_frame_ = declare_parameter<std::string>("global_frame", global_frame_);
  pose_frame_ = declare_parameter<std::string>("pose_frame", pose_frame_);
  compute_mesh_ = declare_parameter<bool>("compute_mesh", compute_mesh_);
  compute_esdf_ = declare_parameter<bool>("compute_esdf", compute_esdf_);
  publish_esdf_distance_slice_ =
    declare_parameter<bool>("publish_esdf_distance_slice", publish_esdf_distance_slice_);
  use_color_ = declare_parameter<bool>("use_color", use_color_);
  use_depth_ = declare_parameter<bool>("use_depth", use_depth_);
  use_lidar_ = declare_parameter<bool>("use_lidar", use_lidar_);

  // Multi mapper params
  voxel_size_ = declare_parameter<float>("voxel_size", voxel_size_);
  mapping_type_ =
    mapping_type_from_string(declare_parameter<std::string>("mapping_type", "static_tsdf"), this);
  esdf_mode_ = declare_parameter<bool>("esdf_2d", true) ? EsdfMode::k2D : EsdfMode::k3D;
  multi_mapper_params_.esdf_2d_min_height =
    declare_parameter<float>("esdf_2d_min_height", MultiMapper::kDefaultEsdf2dMinHeight);
  multi_mapper_params_.esdf_2d_max_height =
    declare_parameter<float>("esdf_2d_max_height", MultiMapper::kDefaultEsdf2dMaxHeight);
  multi_mapper_params_.esdf_slice_height =
    declare_parameter<float>("esdf_slice_height", MultiMapper::kDefaultEsdf2dSliceHeight);
  multi_mapper_params_.connected_mask_component_size_threshold =
    declare_parameter<int>(
    "connected_mask_component_size_threshold",
    MultiMapper::kDefaultConnectedMaskComponentSizeThreshold);

  // Lidar params
  lidar_width_ = declare_parameter<int>("lidar_width", lidar_width_);
  lidar_height_ = declare_parameter<int>("lidar_height", lidar_height_);
  lidar_vertical_fov_rad_ = declare_parameter<float>(
    "lidar_vertical_fov_rad",
    lidar_vertical_fov_rad_);
  use_non_equal_vertical_fov_lidar_params_ = declare_parameter<bool>(
    "use_non_equal_vertical_fov_lidar_params", use_non_equal_vertical_fov_lidar_params_);
  min_angle_below_zero_elevation_rad_ = declare_parameter<float>(
    "min_angle_below_zero_elevation_rad", min_angle_below_zero_elevation_rad_);
  max_angle_above_zero_elevation_rad_ = declare_parameter<float>(
    "max_angle_above_zero_elevation_rad", max_angle_above_zero_elevation_rad_);

  // Slice bound visualization
  slice_visualization_attachment_frame_id_ = declare_parameter<std::string>(
    "slice_visualization_attachment_frame_id", slice_visualization_attachment_frame_id_);
  slice_visualization_side_length_ = declare_parameter<float>(
    "slice_visualization_side_length",
    slice_visualization_side_length_);

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
  static_occupancy_publication_rate_hz_ = declare_parameter<float>(
    "static_occupancy_publication_rate_hz", static_occupancy_publication_rate_hz_);
  dynamic_occupancy_decay_rate_hz_ = declare_parameter<float>(
    "dynamic_occupancy_decay_rate_hz", dynamic_occupancy_decay_rate_hz_);
  max_poll_rate_hz_ =
    declare_parameter<float>("max_poll_rate_hz", max_poll_rate_hz_);

  // Message queue params
  maximum_sensor_message_queue_length_ =
    declare_parameter<int>(
    "maximum_sensor_message_queue_length",
    maximum_sensor_message_queue_length_);

  // Settings for QoS.
  depth_qos_str_ = declare_parameter<std::string>("depth_qos", depth_qos_str_);
  color_qos_str_ = declare_parameter<std::string>("color_qos", color_qos_str_);
  pointcloud_qos_str_ =
    declare_parameter<std::string>("pointcloud_qos", pointcloud_qos_str_);

  // Settings for map clearing
  map_clearing_radius_m_ =
    declare_parameter<float>("map_clearing_radius_m", map_clearing_radius_m_);
  map_clearing_frame_id_ = declare_parameter<std::string>(
    "map_clearing_frame_id", map_clearing_frame_id_);
  clear_outside_radius_rate_hz_ = declare_parameter<float>(
    "clear_outside_radius_rate_hz", clear_outside_radius_rate_hz_);
}

void NvbloxNode::initializeMultiMapper()
{
  // Note: This function needs to be called after getParameters()

  // Create the multi mapper
  // NOTE(remos): Mesh integration is not implemented for occupancy layers.
  multi_mapper_ = std::make_shared<MultiMapper>(
    voxel_size_, mapping_type_, esdf_mode_, MemoryType::kDevice);
  multi_mapper_->setMultiMapperParams(multi_mapper_params_);

  // Get the mapper parameters
  const std::string static_mapper_name = "static_mapper";
  const std::string dynamic_mapper_name = "dynamic_mapper";
  declareMapperParameters(static_mapper_name, this);
  declareMapperParameters(dynamic_mapper_name, this);
  MapperParams static_mapper_params = getMapperParamsFromROS(static_mapper_name, this);
  MapperParams dynamic_mapper_params = getMapperParamsFromROS(dynamic_mapper_name, this);

  // Set the mapper parameters
  multi_mapper_->setMapperParams(static_mapper_params, dynamic_mapper_params);

  // Get direct handles to the underlying mappers
  // NOTE(remos): Ideally, everything would be handled by the multi mapper
  //              and these handles wouldn't be needed.
  static_mapper_ = multi_mapper_.get()->unmasked_mapper();
  dynamic_mapper_ = multi_mapper_.get()->masked_mapper();
}

void NvbloxNode::subscribeToTopics()
{
  RCLCPP_INFO_STREAM(get_logger(), "NvbloxNode::subscribeToTopics()");

  constexpr int kQueueSize = 10;

  if (!use_depth_ && !use_lidar_) {
    RCLCPP_WARN(
      get_logger(),
      "Nvblox is running without depth or lidar input, the cost maps and"
      " reconstructions will not update");
  }

  if (use_depth_) {
    // Subscribe to synchronized depth + cam_info topics
    const auto depth_qos = parseQosString(depth_qos_str_);
    depth_sub_.subscribe(this, "depth/image", depth_qos);
    depth_camera_info_sub_.subscribe(this, "depth/camera_info", depth_qos);

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
    const auto color_qos = parseQosString(color_qos_str_);
    color_sub_.subscribe(this, "color/image", color_qos);
    color_camera_info_sub_.subscribe(this, "color/camera_info", color_qos);

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
    const auto pointcloud_qos = rclcpp::QoS(
      rclcpp::KeepLast(kQueueSize), parseQosString(pointcloud_qos_str_));
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", pointcloud_qos,
      std::bind(
        &NvbloxNode::pointcloudCallback, this,
        std::placeholders::_1));
  }

  // Subscribe to transforms.
  transform_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
    "transform", kQueueSize,
    std::bind(
      &Transformer::transformCallback, &transformer_,
      std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "pose", 10,
    std::bind(
      &Transformer::poseCallback, &transformer_,
      std::placeholders::_1));
}

void NvbloxNode::advertiseTopics()
{
  RCLCPP_INFO_STREAM(get_logger(), "NvbloxNode::advertiseTopics()");
  // Mesh publishers
  mesh_publisher_ = create_publisher<nvblox_msgs::msg::Mesh>("~/mesh", 1);
  mesh_marker_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/mesh_marker", 1);
  // Static esdf
  static_esdf_pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/static_esdf_pointcloud", 1);
  static_map_slice_publisher_ =
    create_publisher<nvblox_msgs::msg::DistanceMapSlice>("~/static_map_slice", 1);
  // Static occupancy
  static_occupancy_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/static_occupancy", 1);
  // Debug outputs
  slice_bounds_publisher_ =
    create_publisher<visualization_msgs::msg::Marker>("~/map_slice_bounds", 1);
  back_projected_depth_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/back_projected_depth", 1);
  // Dynamic occupancy
  dynamic_occupancy_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/dynamic_occupancy", 1);
  // Dynamic esdf
  dynamic_esdf_pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/dynamic_esdf_pointcloud", 1);
  dynamic_map_slice_publisher_ =
    create_publisher<nvblox_msgs::msg::DistanceMapSlice>("~/dynamic_map_slice", 1);
  // Debug output for dynamics
  dynamic_points_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/dynamic_points", 1);
  dynamic_depth_frame_overlay_publisher_ =
    create_publisher<sensor_msgs::msg::Image>("~/dynamic_depth_frame_overlay", 1);
  freespace_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/freespace", 1);
  // Combined esdf
  combined_esdf_pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/combined_esdf_pointcloud", 1);
  combined_map_slice_publisher_ =
    create_publisher<nvblox_msgs::msg::DistanceMapSlice>("~/combined_map_slice", 1);
}

void NvbloxNode::advertiseServices()
{
  RCLCPP_INFO_STREAM(get_logger(), "NvbloxNode::advertiseServices()");

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
  RCLCPP_INFO_STREAM(get_logger(), "NvbloxNode::setupTimers()");
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

  if (isStaticOccupancy(mapping_type_)) {
    occupancy_publishing_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / static_occupancy_publication_rate_hz_),
      std::bind(&NvbloxNode::publishOccupancyPointcloud, this),
      group_processing_);
  }
  if (isUsingDynamicMapper(mapping_type_)) {
    dynamic_occupancy_decay_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / dynamic_occupancy_decay_rate_hz_),
      std::bind(&NvbloxNode::decayDynamicOccupancy, this),
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
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_img_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  printMessageArrivalStatistics(
    *depth_img_ptr, "Depth Statistics",
    &depth_frame_statistics_);
  pushMessageOntoQueue(
    {depth_img_ptr, camera_info_msg}, &depth_image_queue_,
    &depth_queue_mutex_);
}

void NvbloxNode::colorImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & color_image_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  printMessageArrivalStatistics(
    *color_image_ptr, "Color Statistics",
    &rgb_frame_statistics_);
  pushMessageOntoQueue(
    {color_image_ptr, camera_info_msg}, &color_image_queue_,
    &color_queue_mutex_);
}

void NvbloxNode::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud)
{
  printMessageArrivalStatistics(
    *pointcloud, "Pointcloud Statistics",
    &pointcloud_frame_statistics_);
  pushMessageOntoQueue(
    pointcloud, &pointcloud_queue_,
    &pointcloud_queue_mutex_);
}

void NvbloxNode::processDepthQueue()
{
  using ImageInfoMsgPair =
    std::pair<sensor_msgs::msg::Image::ConstSharedPtr,
      sensor_msgs::msg::CameraInfo::ConstSharedPtr>;
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
    std::pair<sensor_msgs::msg::Image::ConstSharedPtr,
      sensor_msgs::msg::CameraInfo::ConstSharedPtr>;
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
  using PointcloudMsg = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
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
  const rclcpp::Time timestamp = get_clock()->now();
  timing::Timer ros_total_timer("ros/total");
  timing::Timer ros_esdf_timer("ros/esdf");

  timing::Timer esdf_integration_timer("ros/esdf/integrate");
  multi_mapper_->updateEsdf();
  esdf_integration_timer.Stop();

  timing::Timer esdf_output_timer("ros/esdf/output");

  sliceAndPublishEsdf(
    "static", static_mapper_,
    static_esdf_pointcloud_publisher_, static_map_slice_publisher_);

  if (isUsingDynamicMapper(mapping_type_)) {
    sliceAndPublishEsdf(
      "dynamic", dynamic_mapper_,
      dynamic_esdf_pointcloud_publisher_, dynamic_map_slice_publisher_);
    sliceAndPublishEsdf(
      "combined_dynamic", static_mapper_,
      combined_esdf_pointcloud_publisher_, combined_map_slice_publisher_, dynamic_mapper_.get());
  }

  // Also publish the slice bounds (showing esdf max/min 2d height)
  if (slice_bounds_publisher_->get_subscription_count() > 0) {
    // The frame to which the slice limits visualization is attached.
    // We get the transform from the plane-body (PB) frame, to the scene (S).
    Transform T_S_PB;
    if (transformer_.lookupTransformToGlobalFrame(
        slice_visualization_attachment_frame_id_, rclcpp::Time(0),
        &T_S_PB))
    {
      // Get and publish the planes representing the slice bounds in z.
      const visualization_msgs::msg::Marker marker = sliceLimitsToMarker(
        T_S_PB, slice_visualization_side_length_, timestamp, global_frame_,
        multi_mapper_params_.esdf_2d_min_height, multi_mapper_params_.esdf_2d_max_height);
      slice_bounds_publisher_->publish(marker);
    } else {
      constexpr float kTimeBetweenDebugMessages = 1.0;
      RCLCPP_INFO_STREAM_THROTTLE(
        get_logger(), *get_clock(), kTimeBetweenDebugMessages,
        "Tried to publish slice bounds but couldn't look up frame: "
          << slice_visualization_attachment_frame_id_);
    }
  }
}

void NvbloxNode::sliceAndPublishEsdf(
  const std::string & name,
  const std::shared_ptr<Mapper> & mapper,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pointcloud_publisher,
  const rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr & slice_publisher,
  const Mapper * mapper_2)
{
  // If anyone wants a slice
  if (pointcloud_publisher->get_subscription_count() > 0 ||
    (publish_esdf_distance_slice_ && slice_publisher->get_subscription_count() > 0))
  {
    // Get the slice as an image
    timing::Timer slicing_timer("ros/" + name + "/esdf/output/compute");
    AxisAlignedBoundingBox aabb;
    Image<float> map_slice_image(MemoryType::kDevice);
    if (mapper_2 != nullptr) {
      esdf_slice_converter_.sliceLayersToCombinedDistanceImage(
        mapper->esdf_layer(), mapper_2->esdf_layer(), multi_mapper_params_.esdf_slice_height,
        &map_slice_image, &aabb);
    } else {
      esdf_slice_converter_.sliceLayerToDistanceImage(
        mapper->esdf_layer(), multi_mapper_params_.esdf_slice_height, &map_slice_image, &aabb);
    }
    slicing_timer.Stop();

    // Publish a pointcloud of the slice image for visualization in RVIZ
    if (pointcloud_publisher->get_subscription_count() > 0) {
      timing::Timer pointcloud_msg_timer("ros/" + name + "/esdf/output/pointcloud");
      sensor_msgs::msg::PointCloud2 pointcloud_msg;
      esdf_slice_converter_.pointcloudMsgFromSliceImage(
        map_slice_image, aabb, multi_mapper_params_.esdf_slice_height,
        mapper->esdf_layer().voxel_size(), &pointcloud_msg);
      pointcloud_msg.header.frame_id = global_frame_;
      pointcloud_msg.header.stamp = get_clock()->now();
      pointcloud_publisher->publish(pointcloud_msg);
    }

    // Publish the distance map slice (costmap for nav2).
    if (publish_esdf_distance_slice_ && slice_publisher->get_subscription_count() > 0) {
      timing::Timer slice_msg_timer("ros/" + name + "/esdf/output/slice");
      nvblox_msgs::msg::DistanceMapSlice map_slice_msg;
      esdf_slice_converter_.distanceMapSliceMsgFromSliceImage(
        map_slice_image, aabb, multi_mapper_params_.esdf_slice_height, mapper->voxel_size_m(),
        &map_slice_msg);
      map_slice_msg.header.frame_id = global_frame_;
      map_slice_msg.header.stamp = get_clock()->now();
      slice_publisher->publish(map_slice_msg);
    }
  }
}

void NvbloxNode::processMesh()
{
  if (!compute_mesh_) {
    return;
  }
  const rclcpp::Time timestamp = get_clock()->now();
  timing::Timer ros_total_timer("ros/total");
  timing::Timer ros_mesh_timer("ros/mesh");

  timing::Timer mesh_integration_timer("ros/mesh/integrate_and_color");
  const std::vector<Index3D> mesh_updated_list = multi_mapper_->updateMesh();
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
    nvblox_msgs::msg::Mesh mesh_msg;
    // In case we have new subscribers, publish the ENTIRE map once.
    if (new_subscriber_count > mesh_subscriber_count_) {
      RCLCPP_INFO(get_logger(), "Got a new subscriber, sending entire map.");
      conversions::meshMessageFromMeshLayer(static_mapper_->mesh_layer(), &mesh_msg);
      mesh_msg.clear = true;
      should_publish = true;
    } else {
      conversions::meshMessageFromMeshBlocks(
        static_mapper_->mesh_layer(),
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
    visualization_msgs::msg::MarkerArray marker_msg;
    conversions::markerMessageFromMeshLayer(
      static_mapper_->mesh_layer(),
      global_frame_, &marker_msg);
    mesh_marker_publisher_->publish(marker_msg);
  }

  mesh_output_timer.Stop();
}

void NvbloxNode::decayDynamicOccupancy() {dynamic_mapper_->decayOccupancy();}

bool NvbloxNode::canTransform(const std_msgs::msg::Header & header)
{
  Transform T_L_C;
  return transformer_.lookupTransformToGlobalFrame(
    header.frame_id,
    header.stamp, &T_L_C);
}

bool NvbloxNode::isUpdateTooFrequent(
  const rclcpp::Time & current_stamp,
  const rclcpp::Time & last_update_stamp,
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
  const std::pair<sensor_msgs::msg::Image::ConstSharedPtr,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr> &
  depth_camera_pair)
{
  timing::Timer ros_depth_timer("ros/depth");
  timing::Timer transform_timer("ros/depth/transform");

  // Message parts
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_img_ptr =
    depth_camera_pair.first;
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg =
    depth_camera_pair.second;

  const rclcpp::Time depth_image_timestamp = depth_img_ptr->header.stamp;

  // Check that we're not updating more quickly than we should.
  if (isUpdateTooFrequent(
      depth_image_timestamp, last_depth_update_time_,
      max_depth_update_hz_))
  {
    return true;
  }
  last_depth_update_time_ = depth_image_timestamp;

  // Get the timestamp
  constexpr int64_t kNanoSecondsToMilliSeconds = 1e6;
  nvblox::Time update_time_ms(
    depth_image_timestamp.nanoseconds() /
    kNanoSecondsToMilliSeconds);

  // Get the TF for this image.
  Transform T_L_C;
  std::string target_frame = depth_img_ptr->header.frame_id;

  if (!transformer_.lookupTransformToGlobalFrame(
      target_frame, depth_image_timestamp, &T_L_C))
  {
    return false;
  }
  transform_timer.Stop();

  timing::Timer conversions_timer("ros/depth/conversions");
  // Convert camera info message to camera object.
  Camera camera = conversions::cameraFromMessage(*camera_info_msg);

  // Convert the depth image.
  if (!conversions::depthImageFromImageMessage(depth_img_ptr, &depth_image_)) {
    RCLCPP_ERROR(get_logger(), "Failed to transform depth image.");
    return false;
  }
  conversions_timer.Stop();

  // Do the actual depth integration
  timing::Timer integration_timer("ros/depth/integrate");
  multi_mapper_->integrateDepth(depth_image_, T_L_C, camera, update_time_ms);
  integration_timer.Stop();

  if (mapping_type_ == MappingType::kDynamic) {
    // Process the freespace layer
    timing::Timer dynamic_publishing_timer("ros/depth/output/dynamics");
    publishFreespace(T_L_C);
    publishDynamics(target_frame);
    dynamic_publishing_timer.Stop();
  }

  // Publish back projected depth image for debugging
  timing::Timer back_projected_depth_timer(
    "ros/depth/output/back_projected_depth");
  if (back_projected_depth_publisher_->get_subscription_count() > 0) {
    publishBackProjectedDepth(camera, T_L_C);
  }
  back_projected_depth_timer.Stop();

  return true;
}

void NvbloxNode::publishFreespace(
  const Transform & T_L_C)
{
  // Publish the freespace layer pointcloud
  if (freespace_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    // Make abb around robot to limit pointcloud size in Rviz.
    constexpr float aabb_size = 5.0f;
    constexpr float abb_min_height = -0.5f;
    constexpr float abb_max_height = 0.5f;
    auto aabb = AxisAlignedBoundingBox(
      Vector3f(-aabb_size, -aabb_size, abb_min_height) + T_L_C.translation(),
      Vector3f(aabb_size, aabb_size, abb_max_height) + T_L_C.translation());
    layer_converter_.pointcloudMsgFromLayerInAABB(
      static_mapper_->freespace_layer(), aabb,
      &pointcloud_msg);
    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = get_clock()->now();
    freespace_publisher_->publish(pointcloud_msg);
  }
}

void NvbloxNode::publishDynamics(
  const std::string & camera_frame_id)
{
  // Publish the dynamic pointcloud
  if (dynamic_points_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 dynamic_points_msg;
    const Pointcloud & dynamic_points =
      multi_mapper_->getLastDynamicPointcloud();
    pointcloud_converter_.pointcloudMsgFromPointcloud(dynamic_points, &dynamic_points_msg);
    dynamic_points_msg.header.frame_id = global_frame_;
    dynamic_points_msg.header.stamp = get_clock()->now();
    dynamic_points_publisher_->publish(dynamic_points_msg);
  }

  // Publish the dynamic overlay
  if (dynamic_depth_frame_overlay_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::Image img_msg;
    const ColorImage & dynamic_overlay =
      multi_mapper_->getLastDynamicFrameMaskOverlay();
    conversions::imageMessageFromColorImage(dynamic_overlay, camera_frame_id, &img_msg);
    dynamic_depth_frame_overlay_publisher_->publish(img_msg);
  }

  // Publish the dynamic occupancy layer
  if (dynamic_occupancy_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    layer_converter_.pointcloudMsgFromLayer(dynamic_mapper_->occupancy_layer(), &pointcloud_msg);
    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = get_clock()->now();
    dynamic_occupancy_publisher_->publish(pointcloud_msg);
  }
}

void NvbloxNode::publishBackProjectedDepth(const Camera & camera, const Transform & T_L_C)
{
  // Get the pointcloud from the depth image
  image_back_projector_.backProjectOnGPU(
    depth_image_, camera,
    &pointcloud_C_device_);
  transformPointcloudOnGPU(T_L_C, pointcloud_C_device_, &pointcloud_L_device_);

  // Send the message
  sensor_msgs::msg::PointCloud2 back_projected_depth_msg;
  pointcloud_converter_.pointcloudMsgFromPointcloud(
    pointcloud_L_device_,
    &back_projected_depth_msg);
  back_projected_depth_msg.header.frame_id = global_frame_;
  back_projected_depth_msg.header.stamp = get_clock()->now();
  back_projected_depth_publisher_->publish(back_projected_depth_msg);
}

bool NvbloxNode::processColorImage(
  const std::pair<sensor_msgs::msg::Image::ConstSharedPtr,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr> &
  color_camera_pair)
{
  timing::Timer ros_color_timer("ros/color");
  timing::Timer transform_timer("ros/color/transform");

  const sensor_msgs::msg::Image::ConstSharedPtr & color_img_ptr =
    color_camera_pair.first;
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg =
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
    RCLCPP_ERROR(get_logger(), "Failed to transform color image.");
    return false;
  }
  color_convert_timer.Stop();

  // Integrate.
  timing::Timer color_integrate_timer("ros/color/integrate");
  multi_mapper_->integrateColor(color_image_, T_L_C, camera);
  color_integrate_timer.Stop();
  return true;
}

bool NvbloxNode::processLidarPointcloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_ptr)
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

  // Create the LiDAR model. The method of creation depends on whether we have
  // equal FoV above and below 0 elevation. This is specified through a param.
  Lidar lidar =
    (use_non_equal_vertical_fov_lidar_params_) ?
    Lidar(
    lidar_width_, lidar_height_,
    min_angle_below_zero_elevation_rad_,
    max_angle_above_zero_elevation_rad_) :
    Lidar(lidar_width_, lidar_height_, lidar_vertical_fov_rad_);

  // We check that the pointcloud is consistent with this LiDAR model
  // NOTE(alexmillane): If the check fails we return true which indicates that
  // this pointcloud can be removed from the queue even though it wasn't
  // integrated (because the intrisics model is messed up).
  // NOTE(alexmillane): Note that internally we cache checks, so each LiDAR
  // intrisics model is only tested against a single pointcloud. This is because
  // the check is expensive to perform.
  if (!pointcloud_converter_.checkLidarPointcloud(pointcloud_ptr, lidar)) {
    RCLCPP_ERROR_ONCE(
      get_logger(),
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

  static_mapper_->integrateLidarDepth(pointcloud_image_, T_L_C, lidar);
  lidar_integration_timer.Stop();

  return true;
}

void NvbloxNode::publishOccupancyPointcloud()
{
  timing::Timer ros_total_timer("ros/total");
  timing::Timer esdf_output_timer("ros/occupancy/output");

  if (static_occupancy_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    layer_converter_.pointcloudMsgFromLayer(static_mapper_->occupancy_layer(), &pointcloud_msg);
    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = get_clock()->now();
    static_occupancy_publisher_->publish(pointcloud_msg);
  }
}

void NvbloxNode::clearMapOutsideOfRadiusOfLastKnownPose()
{
  if (map_clearing_radius_m_ > 0.0f) {
    timing::Timer("ros/clear_outside_radius");
    Transform T_L_MC;  // MC = map clearing frame
    if (transformer_.lookupTransformToGlobalFrame(
        map_clearing_frame_id_,
        rclcpp::Time(0), &T_L_MC))
    {
      const std::vector<Index3D> blocks_cleared = static_mapper_->clearOutsideRadius(
        T_L_MC.translation(), map_clearing_radius_m_);
      // We keep track of the deleted blocks for publishing later.
      mesh_blocks_deleted_.insert(blocks_cleared.begin(), blocks_cleared.end());
    } else {
      constexpr float kTimeBetweenDebugMessages = 1.0;
      RCLCPP_INFO_STREAM_THROTTLE(
        get_logger(), *get_clock(), kTimeBetweenDebugMessages,
        "Tried to clear map outside of radius but couldn't look up frame: "
          << map_clearing_frame_id_);
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
  bool success = true;
  if (ends_with(request->file_path, ".ply")) {
    success =
      io::outputMeshLayerToPly(static_mapper_->mesh_layer(), request->file_path);
  } else {
    // If we get a partial path then output a bunch of stuff to a folder.
    success &= io::outputVoxelLayerToPly(
      static_mapper_->tsdf_layer(),
      request->file_path + "/ros2_tsdf.ply");
    success &= io::outputVoxelLayerToPly(
      static_mapper_->esdf_layer(),
      request->file_path + "/ros2_esdf.ply");
    success &= io::outputMeshLayerToPly(
      static_mapper_->mesh_layer(),
      request->file_path + "/ros2_mesh.ply");
    if (mapping_type_ == MappingType::kDynamic) {
      success &= io::outputVoxelLayerToPly(
        static_mapper_->freespace_layer(),
        request->file_path + "/ros2_freespace.ply");
    }
  }
  if (success) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Output PLY file(s) to " << request->file_path);
    response->success = true;
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
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

  response->success = static_mapper_->saveLayerCake(filename);
  if (response->success) {
    RCLCPP_INFO_STREAM(get_logger(), "Output map to file to " << filename);
  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Failed to write file to " << filename);
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

  response->success = static_mapper_->loadMap(filename);
  if (response->success) {
    RCLCPP_INFO_STREAM(get_logger(), "Loaded map to file from " << filename);
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Failed to load map file from " << filename);
  }
}

}  // namespace nvblox

// Register the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvblox::NvbloxNode)
