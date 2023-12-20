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
#include <nvblox/utils/rates.h>

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
    get_logger(), "Started up nvblox node in frame " <<
      global_frame_ << " and voxel size " <<
      voxel_size_);

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

  // Output debug info
  RCLCPP_INFO_STREAM(
    get_logger(),
    "nvblox parameters:\n" <<
      multi_mapper_->getParametersAsString());
}

void NvbloxNode::getParameters()
{
  // Declare & initialize the parameters.
  RCLCPP_INFO_STREAM(get_logger(), "NvbloxNode::getParameters()");

  // Node params
  global_frame_ = declare_parameter<std::string>("global_frame", global_frame_);
  pose_frame_ = declare_parameter<std::string>("pose_frame", pose_frame_);
  publish_esdf_distance_slice_ = declare_parameter<bool>(
    "publish_esdf_distance_slice", publish_esdf_distance_slice_);
  use_color_ = declare_parameter<bool>("use_color", use_color_);
  use_depth_ = declare_parameter<bool>("use_depth", use_depth_);
  use_lidar_ = declare_parameter<bool>("use_lidar", use_lidar_);

  // Multi mapper params
  voxel_size_ = declare_parameter<float>("voxel_size", voxel_size_);
  mapping_type_ = mapping_type_from_string(
    declare_parameter<std::string>("mapping_type", "static_tsdf"), this);
  esdf_mode_ =
    declare_parameter<bool>("esdf_2d", true) ? EsdfMode::k2D : EsdfMode::k3D;
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
  use_non_equal_vertical_fov_lidar_params_ =
    declare_parameter<bool>(
    "use_non_equal_vertical_fov_lidar_params",
    use_non_equal_vertical_fov_lidar_params_);
  min_angle_below_zero_elevation_rad_ =
    declare_parameter<float>(
    "min_angle_below_zero_elevation_rad",
    min_angle_below_zero_elevation_rad_);
  max_angle_above_zero_elevation_rad_ =
    declare_parameter<float>(
    "max_angle_above_zero_elevation_rad",
    max_angle_above_zero_elevation_rad_);

  // Slice bound visualization
  slice_visualization_attachment_frame_id_ =
    declare_parameter<std::string>(
    "slice_visualization_attachment_frame_id",
    slice_visualization_attachment_frame_id_);
  slice_visualization_side_length_ = declare_parameter<float>(
    "slice_visualization_side_length", slice_visualization_side_length_);

  // Update rates
  integrate_depth_rate_hz_ =
    declare_parameter<float>("integrate_depth_rate_hz", integrate_depth_rate_hz_);
  integrate_color_rate_hz_ =
    declare_parameter<float>("integrate_color_rate_hz", integrate_color_rate_hz_);
  integrate_lidar_rate_hz_ =
    declare_parameter<float>("integrate_lidar_rate_hz", integrate_lidar_rate_hz_);
  update_mesh_rate_hz_ =
    declare_parameter<float>("update_mesh_rate_hz", update_mesh_rate_hz_);
  update_esdf_rate_hz_ =
    declare_parameter<float>("update_esdf_rate_hz", update_esdf_rate_hz_);
  publish_static_occupancy_rate_hz_ =
    declare_parameter<float>("publish_static_occupancy_rate_hz", publish_static_occupancy_rate_hz_);
  decay_tsdf_rate_hz_ =
    declare_parameter<float>("decay_tsdf_rate_hz", decay_tsdf_rate_hz_);
  decay_dynamic_occupancy_rate_hz_ =
    declare_parameter<float>("decay_dynamic_occucancy_rate_hz", decay_dynamic_occupancy_rate_hz_);
  clear_map_outside_radius_rate_hz_ =
    declare_parameter<float>("clear_map_outside_radius_rate_hz", clear_map_outside_radius_rate_hz_);
  tick_period_ms_ =
    declare_parameter<int>("tick_period_ms", tick_period_ms_);
  print_statistics_on_console_period_ms_ =
    declare_parameter<int>(
    "print_statistics_on_console_period_ms",
    print_statistics_on_console_period_ms_);
  print_timings_to_console_ =
    declare_parameter<bool>("print_timings_to_console", print_timings_to_console_);
  print_rates_to_console_ =
    declare_parameter<bool>("print_rates_to_console", print_rates_to_console_);

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

  // Visualization params
  max_back_projection_distance_ = declare_parameter<float>(
    "max_back_projection_distance", max_back_projection_distance_);
  back_projection_subsampling_ = declare_parameter<int>(
    "back_projection_subsampling", back_projection_subsampling_);
  enable_mesh_markers_ = declare_parameter<bool>(
    "enable_mesh_markers", enable_mesh_markers_);
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
  MapperParams static_mapper_params =
    getMapperParamsFromROS(static_mapper_name, this);
  MapperParams dynamic_mapper_params =
    getMapperParamsFromROS(dynamic_mapper_name, this);

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
    auto depth_callback = std::bind(
      &NvbloxNode::depthImageCallback, this,
      std::placeholders::_1, std::placeholders::_2);

    static_assert(
      // workaround since depth_subs_.size() is not constexpr()
      std::tuple_size<decltype(depth_subs_)>::value == kDepthTopicBaseNames.size(),
      "Array sizes must match");
    const auto depth_qos = parseQosString(depth_qos_str_);
    for (size_t i = 0; i < depth_subs_.size(); ++i) {
      depth_subs_[i].subscribe(this, depth_qos, kDepthTopicBaseNames[i], depth_callback);
    }
  }

  if (use_color_) {
    // Subscribe to synchronized color + cam_info topics
    auto color_callback = std::bind(
      &NvbloxNode::colorImageCallback, this,
      std::placeholders::_1, std::placeholders::_2);

    const auto color_qos = parseQosString(color_qos_str_);
    static_assert(
      std::tuple_size<decltype(color_subs_)>::value == kColorTopicBaseNames.size(),
      "Array sizes must match");

    for (size_t i = 0; i < color_subs_.size(); ++i) {
      color_subs_[i].subscribe(this, color_qos, kColorTopicBaseNames[i], color_callback);
    }
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
  if (enable_mesh_markers_) {
    mesh_marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/mesh_marker", 1);
  }
  // Static esdf
  static_esdf_pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/static_esdf_pointcloud", 1);
  static_map_slice_publisher_ =
    create_publisher<nvblox_msgs::msg::DistanceMapSlice>(
    "~/static_map_slice",
    1);
  if (isStaticOccupancy(mapping_type_)) {
    // Static occupancy
    static_occupancy_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("~/static_occupancy", 1);
  }
  // Debug outputs
  slice_bounds_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
    "~/map_slice_bounds", 1);
  back_projected_depth_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/back_projected_depth",
    1);
  if (isDynamicMapping(mapping_type_)) {
    // Dynamic occupancy
    dynamic_occupancy_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("~/dynamic_occupancy", 1);
    // Debug output for dynamics
    dynamic_points_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("~/dynamic_points", 1);
    dynamic_depth_frame_overlay_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(
      "~/dynamic_depth_frame_overlay", 1);
    freespace_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("~/freespace", 1);
  }
  if (isUsingBothMappers(mapping_type_)) {
    // Dynamic esdf
    dynamic_esdf_pointcloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/dynamic_esdf_pointcloud", 1);
    dynamic_map_slice_publisher_ =
      create_publisher<nvblox_msgs::msg::DistanceMapSlice>(
      "~/dynamic_map_slice", 1);
    // Combined esdf
    combined_esdf_pointcloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/combined_esdf_pointcloud", 1);
    combined_map_slice_publisher_ =
      create_publisher<nvblox_msgs::msg::DistanceMapSlice>(
      "~/combined_map_slice", 1);
  }
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
  save_rates_service_ = create_service<nvblox_msgs::srv::FilePath>(
    "~/save_rates",
    std::bind(
      &NvbloxNode::saveRates, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, group_processing_);
  save_timings_service_ = create_service<nvblox_msgs::srv::FilePath>(
    "~/save_timings",
    std::bind(
      &NvbloxNode::saveTimings, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, group_processing_);
}

void NvbloxNode::setupTimers()
{
  RCLCPP_INFO_STREAM(get_logger(), "NvbloxNode::setupTimers()");

  queue_processing_timer_ = create_wall_timer(
    std::chrono::duration<double>(tick_period_ms_ / 1000.0),
    std::bind(&NvbloxNode::tick, this), group_processing_);
}

void NvbloxNode::depthImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_img_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  timing::Timer tick_timer("ros/depth_image_callback");
  printMessageArrivalStatistics(
    *depth_img_ptr, "Depth Statistics",
    &depth_frame_statistics_);
  pushMessageOntoQueue(
    "depth_queue",
    {depth_img_ptr, camera_info_msg}, &depth_image_queue_,
    &depth_queue_mutex_);
}

void NvbloxNode::colorImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & color_image_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  timing::Timer tick_timer("ros/color_image_callback");
  printMessageArrivalStatistics(
    *color_image_ptr, "Color Statistics",
    &rgb_frame_statistics_);
  pushMessageOntoQueue(
    "color_queue",
    {color_image_ptr, camera_info_msg}, &color_image_queue_,
    &color_queue_mutex_);
}

void NvbloxNode::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud)
{
  timing::Timer tick_timer("ros/pointcloud_callback");
  printMessageArrivalStatistics(
    *pointcloud, "Pointcloud Statistics",
    &pointcloud_frame_statistics_);
  pushMessageOntoQueue(
    "pointcloud_queue",
    pointcloud, &pointcloud_queue_,
    &pointcloud_queue_mutex_);
}

bool NvbloxNode::shouldProcess(
  const rclcpp::Time & time_now, const rclcpp::Time & time_last,
  const float desired_frequency_hz)
{
  rclcpp::Duration time_since_last = time_now - time_last;
  return time_since_last.seconds() > (1.0 / desired_frequency_hz);
}

void NvbloxNode::tick()
{
  // The idle timer measures time spent *outside* the tick function
  idle_timer_.Stop();

  timing::Timer tick_timer("ros/tick");
  timing::Rates::tick("ros/tick");

  // Process sensor data
  // NOTE: We process these queues every time, checking if we can process (or discard) messages
  // in the queue. Dropping messages in order to not exceed integration rates is handled inside
  // the processQueue functions.
  if (use_depth_) {
    processDepthQueue();
  }
  if (use_color_) {
    processColorQueue();
  }
  if (use_lidar_) {
    processPointcloudQueue();
  }

  // Decay
  if (const rclcpp::Time now = this->get_clock()->now();
    shouldProcess(now, decay_tsdf_last_time_, decay_tsdf_rate_hz_))
  {
    decayTsdf();
    decay_tsdf_last_time_ = now;
  }
  if (const rclcpp::Time now = this->get_clock()->now();
    shouldProcess(now, decay_dynamic_occupancy_last_time_, decay_dynamic_occupancy_rate_hz_))
  {
    decayDynamicOccupancy();
    decay_dynamic_occupancy_last_time_ = now;
  }

  // Mapping
  if (const rclcpp::Time now = this->get_clock()->now();
    shouldProcess(now, clear_map_outside_radius_last_time_, clear_map_outside_radius_rate_hz_))
  {
    clearMapOutsideOfRadiusOfLastKnownPose();
    clear_map_outside_radius_last_time_ = now;
  }

  // Output cost map
  if (const rclcpp::Time now = this->get_clock()->now();
    shouldProcess(now, update_esdf_last_time_, update_esdf_rate_hz_))
  {
    processEsdf();
    update_esdf_last_time_ = now;
  }

  // Visualization
  if (const rclcpp::Time now = this->get_clock()->now();
    shouldProcess(now, update_mesh_last_time_, update_mesh_rate_hz_))
  {
    processMesh();
    update_mesh_last_time_ = now;
  }
  if (const rclcpp::Time now = this->get_clock()->now(); isStaticOccupancy(mapping_type_) &&
    shouldProcess(now, publish_static_occupancy_last_time_, publish_static_occupancy_rate_hz_))
  {
    publishStaticOccupancyPointcloud();
    publish_static_occupancy_last_time_ = now;
  }

  // Restart the idle timer
  idle_timer_.Start();
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
}

void NvbloxNode::processEsdf()
{
  const rclcpp::Time timestamp = get_clock()->now();
  timing::Timer ros_esdf_timer("ros/esdf");
  timing::Rates::tick("ros/update_esdf");

  timing::Timer esdf_integration_timer("ros/esdf/integrate");
  multi_mapper_->updateEsdf();
  esdf_integration_timer.Stop();

  timing::Timer esdf_output_timer("ros/esdf/output");

  sliceAndPublishEsdf(
    "static", static_mapper_,
    static_esdf_pointcloud_publisher_,
    static_map_slice_publisher_);

  if (isUsingBothMappers(mapping_type_)) {
    sliceAndPublishEsdf(
      "dynamic", dynamic_mapper_,
      dynamic_esdf_pointcloud_publisher_,
      dynamic_map_slice_publisher_);
    sliceAndPublishEsdf(
      "combined_dynamic", static_mapper_,
      combined_esdf_pointcloud_publisher_,
      combined_map_slice_publisher_, dynamic_mapper_.get());
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
      const visualization_msgs::msg::Marker marker_bottom = sliceLimitsToMarker(
        T_S_PB, slice_visualization_side_length_, timestamp, global_frame_,
        static_mapper_->esdf_slice_min_height(),
        SliceLimitMarkerType::kBottomSliceLimit);
      const visualization_msgs::msg::Marker marker_top = sliceLimitsToMarker(
        T_S_PB, slice_visualization_side_length_, timestamp, global_frame_,
        static_mapper_->esdf_slice_max_height(),
        SliceLimitMarkerType::kTopSliceLimit);
      slice_bounds_publisher_->publish(marker_bottom);
      slice_bounds_publisher_->publish(marker_top);
    } else {
      constexpr float kTimeBetweenDebugMessages = 1.0;
      RCLCPP_INFO_STREAM_THROTTLE(
        get_logger(), *get_clock(), kTimeBetweenDebugMessages,
        "Tried to publish slice bounds but couldn't look up frame: " <<
          slice_visualization_attachment_frame_id_);
    }
  }
}

void NvbloxNode::sliceAndPublishEsdf(
  const std::string & name, const std::shared_ptr<Mapper> & mapper,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &
  pointcloud_publisher,
  const rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr &
  slice_publisher,
  const Mapper * mapper_2)
{
  // If anyone wants a slice
  if (pointcloud_publisher->get_subscription_count() > 0 ||
    (publish_esdf_distance_slice_ &&
    slice_publisher->get_subscription_count() > 0))
  {
    // Get the slice as an image
    timing::Timer slicing_timer("ros/" + name + "/esdf/output/compute");
    AxisAlignedBoundingBox aabb;
    Image<float> map_slice_image(MemoryType::kDevice);
    if (mapper_2 != nullptr) {
      esdf_slice_converter_.sliceLayersToCombinedDistanceImage(
        mapper->esdf_layer(), mapper_2->esdf_layer(),
        mapper->esdf_slice_height(), mapper_2->esdf_slice_height(),
        &map_slice_image, &aabb);
    } else {
      esdf_slice_converter_.sliceLayerToDistanceImage(
        mapper->esdf_layer(), mapper->esdf_slice_height(), &map_slice_image,
        &aabb);
    }
    slicing_timer.Stop();

    // Publish a pointcloud of the slice image for visualization in RVIZ
    if (pointcloud_publisher->get_subscription_count() > 0) {
      timing::Timer pointcloud_msg_timer("ros/" + name +
        "/esdf/output/pointcloud");
      sensor_msgs::msg::PointCloud2 pointcloud_msg;
      esdf_slice_converter_.pointcloudMsgFromSliceImage(
        map_slice_image, aabb, mapper->esdf_slice_height(),
        mapper->esdf_layer().voxel_size(), &pointcloud_msg);
      pointcloud_msg.header.frame_id = global_frame_;
      pointcloud_msg.header.stamp = get_clock()->now();
      pointcloud_publisher->publish(pointcloud_msg);
    }

    // Publish the distance map slice (costmap for nav2).
    if (publish_esdf_distance_slice_ &&
      slice_publisher->get_subscription_count() > 0)
    {
      timing::Timer slice_msg_timer("ros/" + name + "/esdf/output/slice");
      nvblox_msgs::msg::DistanceMapSlice map_slice_msg;
      esdf_slice_converter_.distanceMapSliceMsgFromSliceImage(
        map_slice_image, aabb, mapper->esdf_slice_height(),
        mapper->voxel_size_m(), &map_slice_msg);
      map_slice_msg.header.frame_id = global_frame_;
      map_slice_msg.header.stamp = get_clock()->now();
      slice_publisher->publish(map_slice_msg);
    }
  }
}

void NvbloxNode::processMesh()
{
  const rclcpp::Time timestamp = get_clock()->now();
  timing::Timer ros_mesh_timer("ros/mesh");

  timing::Timer mesh_integration_timer("ros/mesh/integrate_and_color");

  Transform T_L_C;
  if (!transformer_.lookupTransformToGlobalFrame(
      map_clearing_frame_id_,
      rclcpp::Time(0), &T_L_C))
  {
    RCLCPP_WARN_STREAM(
      get_logger(), "Lookup transform failed for frame " <<
        map_clearing_frame_id_ <<
        ". Mesh not published");
    return;
  }

  std::shared_ptr<const SerializedMesh> serialized_mesh =
    multi_mapper_->updateMesh(T_L_C);
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "Num of streamed mesh blocks: " << serialized_mesh->block_indices.size());

  // In the case that some mesh blocks have been re-added after deletion, remove
  // them from the deleted list.
  for (const Index3D & idx : serialized_mesh->block_indices) {
    mesh_blocks_deleted_.erase(idx);
  }
  // Make a list to be published to rviz of blocks to be removed from the viz
  const std::vector<Index3D> mesh_blocks_to_delete(mesh_blocks_deleted_.begin(),
    mesh_blocks_deleted_.end());
  mesh_blocks_deleted_.clear();

  // Publish the mesh updates.
  timing::Timer mesh_output_timer("ros/mesh/output");
  size_t new_subscriber_count = mesh_publisher_->get_subscription_count();
  if (new_subscriber_count > 0) {
    nvblox_msgs::msg::Mesh mesh_msg;
    // In case we have new subscribers, publish the ENTIRE map once.
    if (new_subscriber_count > mesh_subscriber_count_) {
      RCLCPP_INFO(get_logger(), "Got a new subscriber, sending entire map.");
      conversions::meshMessageFromMeshLayer(
        static_mapper_->mesh_layer(), timestamp, global_frame_,
        &mesh_msg);
      mesh_publisher_->publish(mesh_msg);
    } else {
      // Delete mesh blocks
      if (!mesh_blocks_to_delete.empty()) {
        nvblox_msgs::msg::Mesh deleted_mesh_msg;
        conversions::meshMessageFromBlocksToDelete(
          mesh_blocks_to_delete, timestamp, global_frame_,
          static_mapper_->mesh_layer().block_size(), &deleted_mesh_msg);
        mesh_publisher_->publish(deleted_mesh_msg);
      }

      // Publish mesh blocks from serialized mesh
      if (!serialized_mesh->block_indices.empty()) {
        conversions::meshMessageFromSerializedMesh(
          serialized_mesh, timestamp, global_frame_, static_mapper_->mesh_layer().block_size(),
          &mesh_msg);
        mesh_publisher_->publish(mesh_msg);
        timing::Rates::tick("ros/mesh");
      }
    }
  }
  mesh_subscriber_count_ = new_subscriber_count;

  // optionally publish the markers.
  if (enable_mesh_markers_ &&
    mesh_marker_publisher_->get_subscription_count() > 0)
  {
    visualization_msgs::msg::MarkerArray marker_msg;
    conversions::markerMessageFromSerializedMesh(
      serialized_mesh, global_frame_,
      &marker_msg);
    mesh_marker_publisher_->publish(marker_msg);
  }

  mesh_output_timer.Stop();
}

void NvbloxNode::decayDynamicOccupancy()
{
  timing::Timer timer("ros/decay_dynamic_occupancy");
  dynamic_mapper_->decayOccupancy();
}

void NvbloxNode::decayTsdf()
{
  timing::Timer timer("ros/decay_tsdf");
  const std::vector<Index3D> deallocated_blocks = static_mapper_->decayTsdf();
  mesh_blocks_deleted_.insert(
    deallocated_blocks.begin(),
    deallocated_blocks.end());
}

bool NvbloxNode::canTransform(const std_msgs::msg::Header & header)
{
  Transform T_L_C;
  return transformer_.lookupTransformToGlobalFrame(
    header.frame_id,
    header.stamp, &T_L_C);
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

  // Check if the message is too soon, and if it is discard it
  const rclcpp::Time depth_image_timestamp = depth_img_ptr->header.stamp;
  if (!shouldProcess(depth_image_timestamp, integrate_depth_last_time_, integrate_depth_rate_hz_)) {
    // To discard we indicate that the image was processed, without actually integrating it.
    return true;
  }

  // Get the timestamp
  constexpr int64_t kNanoSecondsToMilliSeconds = 1e6;
  nvblox::Time update_time_ms(depth_image_timestamp.nanoseconds() /
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
  timing::Rates::tick("ros/depth");
  multi_mapper_->integrateDepth(depth_image_, T_L_C, camera, update_time_ms);
  integrate_depth_last_time_ = depth_image_timestamp;
  integration_timer.Stop();

  // Optional publishing - Freespace + Dynamics
  if (isDynamicMapping(mapping_type_)) {
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
    // Only back project and publish every n-th depth image
    if (back_projection_idx_ % back_projection_subsampling_ == 0) {
      publishBackProjectedDepth(camera, T_L_C);
    }
    back_projection_idx_++;
  }
  back_projected_depth_timer.Stop();

  return true;
}

void NvbloxNode::publishFreespace(const Transform & T_L_C)
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
      static_mapper_->freespace_layer(), aabb, &pointcloud_msg);
    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = get_clock()->now();
    freespace_publisher_->publish(pointcloud_msg);
  }
}

void NvbloxNode::publishDynamics(const std::string & camera_frame_id)
{
  // Publish the dynamic pointcloud
  if (dynamic_points_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 dynamic_points_msg;
    const Pointcloud & dynamic_points =
      multi_mapper_->getLastDynamicPointcloud();
    pointcloud_converter_.pointcloudMsgFromPointcloud(
      dynamic_points,
      &dynamic_points_msg);
    dynamic_points_msg.header.frame_id = global_frame_;
    dynamic_points_msg.header.stamp = get_clock()->now();
    dynamic_points_publisher_->publish(dynamic_points_msg);
  }

  // Publish the dynamic overlay
  if (dynamic_depth_frame_overlay_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::Image img_msg;
    const ColorImage & dynamic_overlay =
      multi_mapper_->getLastDynamicFrameMaskOverlay();
    conversions::imageMessageFromColorImage(
      dynamic_overlay, camera_frame_id,
      &img_msg);
    dynamic_depth_frame_overlay_publisher_->publish(img_msg);
  }

  // Publish the dynamic occupancy layer
  if (dynamic_occupancy_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    layer_converter_.pointcloudMsgFromLayer(
      dynamic_mapper_->occupancy_layer(),
      &pointcloud_msg);
    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = get_clock()->now();
    dynamic_occupancy_publisher_->publish(pointcloud_msg);
  }
}

void NvbloxNode::publishBackProjectedDepth(
  const Camera & camera,
  const Transform & T_L_C)
{
  // Get the pointcloud from the depth image
  image_back_projector_.backProjectOnGPU(
    depth_image_, camera,
    &pointcloud_C_device_,
    max_back_projection_distance_);
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

  // Check if the message is too soon, and if it is discard it
  const rclcpp::Time color_image_timestamp = color_img_ptr->header.stamp;
  if (!shouldProcess(color_image_timestamp, integrate_color_last_time_, integrate_color_rate_hz_)) {
    // To discard we indicate that the image was processed, without actually integrating it.
    return true;
  }

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
  timing::Rates::tick("ros/color");
  multi_mapper_->integrateColor(color_image_, T_L_C, camera);
  integrate_color_last_time_ = color_image_timestamp;
  color_integrate_timer.Stop();
  return true;
}

bool NvbloxNode::processLidarPointcloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_ptr)
{
  timing::Timer ros_lidar_timer("ros/lidar");
  timing::Timer transform_timer("ros/lidar/transform");
  timing::Rates::tick("ros/lidar");

  // Check if the message is too soon, and if it is discard it
  const rclcpp::Time pointcloud_timestamp = pointcloud_ptr->header.stamp;
  if (!shouldProcess(pointcloud_timestamp, integrate_lidar_last_time_, integrate_lidar_rate_hz_)) {
    // To discard we indicate that the image was processed, without actually integrating it.
    return true;
  }

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
  integrate_lidar_last_time_ = pointcloud_timestamp;
  lidar_integration_timer.Stop();

  return true;
}

void NvbloxNode::publishStaticOccupancyPointcloud()
{
  timing::Timer esdf_output_timer("ros/occupancy/output");

  if (static_occupancy_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    layer_converter_.pointcloudMsgFromLayer(
      static_mapper_->occupancy_layer(),
      &pointcloud_msg);
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
      const std::vector<Index3D> blocks_cleared =
        static_mapper_->clearOutsideRadius(
        T_L_MC.translation(),
        map_clearing_radius_m_);
      // We keep track of the deleted blocks for publishing later.
      mesh_blocks_deleted_.insert(blocks_cleared.begin(), blocks_cleared.end());
    } else {
      constexpr float kTimeBetweenDebugMessages = 1.0;
      RCLCPP_INFO_STREAM_THROTTLE(
        get_logger(), *get_clock(), kTimeBetweenDebugMessages,
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
  bool success = true;
  if (ends_with(request->file_path, ".ply")) {
    success = io::outputMeshLayerToPly(
      static_mapper_->mesh_layer(),
      request->file_path);
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
    if (isDynamicMapping(mapping_type_)) {
      success &=
        io::outputVoxelLayerToPly(
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

void NvbloxNode::saveRates(
  const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
  std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response)
{
  // If we get a full path, then write to that path.
  bool success = false;
  if (ends_with(request->file_path, ".txt")) {
    std::ofstream rates_file(request->file_path, std::ofstream::out);
    rates_file << nvblox::timing::Rates::Print();
    rates_file.close();
    success = true;
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The rates file should be a text file!!!");
  }
  if (success) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Output rates to " << request->file_path);
    response->success = true;
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Failed to write rates to " << request->file_path);
    response->success = false;
  }
}

void NvbloxNode::saveTimings(
  const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
  std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response)
{
  // If we get a full path, then write to that path.
  bool success = false;
  if (ends_with(request->file_path, ".txt")) {
    std::ofstream timing_file(request->file_path, std::ofstream::out);
    timing_file << nvblox::timing::Timing::Print();
    timing_file.close();
    success = true;
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The timings file should be a text file!!!");
  }
  if (success) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Output timings to " << request->file_path);
    response->success = true;
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Failed to write timings to " << request->file_path);
    response->success = false;
  }
}

}  // namespace nvblox

// Register the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvblox::NvbloxNode)
