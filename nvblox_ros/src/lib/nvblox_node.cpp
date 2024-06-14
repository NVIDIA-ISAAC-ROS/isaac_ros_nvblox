// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <nvblox/core/parameter_tree.h>
#include <nvblox/io/mesh_io.h>
#include <nvblox/io/pointcloud_io.h>
#include <nvblox/utils/delays.h>
#include <nvblox/utils/rates.h>

#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <isaac_ros_common/qos.hpp>

#include "nvblox_ros/conversions/esdf_and_gradients_conversions.hpp"
#include "nvblox_ros/visualization.hpp"

namespace nvblox
{

NvbloxNode::NvbloxNode(const rclcpp::NodeOptions & options, const std::string & node_name)
: Node(node_name, options), transformer_(this)
{
  // Get parameters first (stuff below depends on parameters)
  initializeRosParams(this, &params_, &parameter_tree_);
  CHECK_LE(params_.num_cameras, kMaxNumCameras);

  multi_mapper_params_.connected_mask_component_size_threshold =
    declare_parameter<int>(
    "connected_mask_component_size_threshold",
    MultiMapper::kDefaultConnectedMaskComponentSizeThreshold);

  // Set the transformer settings.
  transformer_.set_global_frame(params_.global_frame.get());
  transformer_.set_pose_frame(params_.pose_frame.get());

  // Create callback groups, which allows processing to go in parallel with the
  // subscriptions.
  group_processing_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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
    get_logger(), "Started up nvblox node in frame " << params_.global_frame.get()
                                                     << " and voxel size "
                                                     << params_.voxel_size);

  // Check if a valid mapping typ was selected
  RCLCPP_INFO_STREAM(get_logger(), "Mapping type: " << toString(params_.mapping_type));
  if (get_name() == std::string("nvblox_node")) {
    if (isHumanMapping(params_.mapping_type)) {
      RCLCPP_FATAL_STREAM(
        get_logger(),
        "Invalid option. The basic nvblox node can not do human mapping. "
        "Use the nvblox human node for that. Exiting nvblox.");
      exit(1);
    }
  }

  // Output debug info
  CHECK(parameter_tree_.children().has_value());
  parameter_tree_.children().value().push_back(multi_mapper_->getParameterTree());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "nvblox parameters:\n"
      << nvblox::parameters::parameterTreeToString(parameter_tree_));
}

NvbloxNode::~NvbloxNode()
{
  RCLCPP_INFO_STREAM(get_logger(), "Timing statistics: \n" << nvblox::timing::Timing::Print());
  RCLCPP_INFO_STREAM(get_logger(), "Rates statistics: \n" << nvblox::timing::Rates::Print());
  RCLCPP_INFO_STREAM(get_logger(), "Delay statistics: \n" << nvblox::timing::Delays::Print());
}

void NvbloxNode::initializeMultiMapper()
{
  // Note: This function needs to be called after getParameters()

  // Create the multi mapper
  // NOTE(remos): Mesh integration is not implemented for occupancy layers.
  multi_mapper_ = std::make_shared<MultiMapper>(
    params_.voxel_size, params_.mapping_type,
    params_.esdf_mode, MemoryType::kDevice,
    std::make_shared<nvblox::CudaStream>(cuda_stream_));
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

  if (!params_.use_depth && !params_.use_lidar) {
    RCLCPP_WARN(
      get_logger(), "Nvblox is running without depth or lidar input, the cost maps and"
      " reconstructions will not update");
  }

  // Create this to skip collecting front camera images incase human segmentation is on
  // Note(Vik): This is a temporary fix. Human mapping should ideally stay in human mapping node
  // or everything should come to this node
  size_t camera_index = 0;
  if (isHumanMapping(params_.mapping_type)) {
    // We skip camera 0 as it will be processed by the human node
    camera_index = 1;
  }

  // Settings for QoS.
  const rclcpp::QoS input_qos =
    isaac_ros::common::AddQosParameter(*this, kDefaultInputQos_, "input_qos");
  std::string input_qos_str = kDefaultInputQos_;
  get_parameter("input_qos", input_qos_str);
  RCLCPP_INFO_STREAM(get_logger(), "Subscribing input topics with QoS: " << input_qos_str);

  if (params_.use_depth) {
    for (int i = camera_index; i < params_.num_cameras; ++i) {
      const std::string base_name(kDepthTopicBaseNames[i]);
      camera_info_subs_.emplace_back(
        create_subscription<sensor_msgs::msg::CameraInfo>(
          base_name + "/camera_info", input_qos,
          std::bind(&CameraCache::update, &depth_camera_cache_, std::placeholders::_1)));

      nitros_image_subs_.emplace_back(
        std::make_shared<NitrosViewSubscriber>(
          this, base_name + "/image",
          nvidia::isaac_ros::nitros::nitros_image_32FC1_t::supported_type_name,
          std::bind(&NvbloxNode::depthImageCallback, this, std::placeholders::_1),
          nvidia::isaac_ros::nitros::NitrosStatisticsConfig(), input_qos));
    }
  }
  if (params_.use_color) {
    for (int i = camera_index; i < params_.num_cameras; ++i) {
      const std::string base_name(kColorTopicBaseNames[i]);
      camera_info_subs_.emplace_back(
        create_subscription<sensor_msgs::msg::CameraInfo>(
          base_name + "/camera_info", input_qos,
          std::bind(&CameraCache::update, &color_camera_cache_, std::placeholders::_1)));

      nitros_image_subs_.emplace_back(
        std::make_shared<NitrosViewSubscriber>(
          this, base_name + "/image",
          nvidia::isaac_ros::nitros::nitros_image_rgb8_t::supported_type_name,
          std::bind(&NvbloxNode::colorImageCallback, this, std::placeholders::_1),
          nvidia::isaac_ros::nitros::NitrosStatisticsConfig(), input_qos));
    }
  }

  if (params_.use_lidar) {
    // Subscribe to pointclouds.
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", input_qos,
      std::bind(&NvbloxNode::pointcloudCallback, this, std::placeholders::_1));
  }

  // Subscribe to transforms.
  transform_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
    "transform", kQueueSize,
    std::bind(&Transformer::transformCallback, &transformer_, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "pose", 10, std::bind(&Transformer::poseCallback, &transformer_, std::placeholders::_1));
}

void NvbloxNode::advertiseTopics()
{
  RCLCPP_INFO_STREAM(get_logger(), "NvbloxNode::advertiseTopics()");
  // Mesh publishers
  mesh_publisher_ = create_publisher<nvblox_msgs::msg::Mesh>("~/mesh", 1);
  if (params_.enable_mesh_markers) {
    mesh_marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/mesh_marker", 1);
  }

  // Layer publishers
  if (!isStaticOccupancy(params_.mapping_type)) {
    tsdf_layer_publisher_ = create_publisher<visualization_msgs::msg::Marker>("~/tsdf_layer", 1);
  }
  color_layer_publisher_ = create_publisher<visualization_msgs::msg::Marker>("~/color_layer", 1);
  if (isStaticOccupancy(params_.mapping_type)) {
    occupancy_layer_publisher_ =
      create_publisher<visualization_msgs::msg::Marker>("~/occupancy_layer", 1);
  }
  if (isDynamicMapping(params_.mapping_type)) {
    freespace_layer_publisher_ =
      create_publisher<visualization_msgs::msg::Marker>("~/freespace_layer", 1);
  }

  // Static esdf
  static_esdf_pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/static_esdf_pointcloud", 1);
  static_map_slice_publisher_ =
    create_publisher<nvblox_msgs::msg::DistanceMapSlice>("~/static_map_slice", 1);
  // Debug outputs
  slice_bounds_publisher_ =
    create_publisher<visualization_msgs::msg::Marker>("~/map_slice_bounds", 1);
  back_projected_depth_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/back_projected_depth", 1);
  if (isDynamicMapping(params_.mapping_type) || isHumanMapping(params_.mapping_type)) {
    // Dynamic occupancy
    dynamic_occupancy_layer_publisher_ =
      create_publisher<visualization_msgs::msg::Marker>("~/dynamic_occupancy_layer", 1);
    // Debug output for dynamics
    dynamic_points_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("~/dynamic_points", 1);
    dynamic_depth_frame_overlay_publisher_ =
      create_publisher<sensor_msgs::msg::Image>("~/dynamic_depth_frame_overlay", 1);
  }
  if (isUsingBothMappers(params_.mapping_type)) {
    // Dynamic esdf
    dynamic_esdf_pointcloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("~/dynamic_esdf_pointcloud", 1);
    dynamic_map_slice_publisher_ =
      create_publisher<nvblox_msgs::msg::DistanceMapSlice>("~/dynamic_map_slice", 1);
    // Combined esdf
    combined_esdf_pointcloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("~/combined_esdf_pointcloud", 1);
    combined_map_slice_publisher_ =
      create_publisher<nvblox_msgs::msg::DistanceMapSlice>("~/combined_map_slice", 1);
  }
}

void NvbloxNode::advertiseServices()
{
  RCLCPP_INFO_STREAM(get_logger(), "NvbloxNode::advertiseServices()");

  save_ply_service_ = create_service<nvblox_msgs::srv::FilePath>(
    "~/save_ply",
    std::bind(&NvbloxNode::savePly, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, group_processing_);
  save_map_service_ = create_service<nvblox_msgs::srv::FilePath>(
    "~/save_map",
    std::bind(&NvbloxNode::saveMap, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, group_processing_);
  load_map_service_ = create_service<nvblox_msgs::srv::FilePath>(
    "~/load_map",
    std::bind(&NvbloxNode::loadMap, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, group_processing_);
  save_rates_service_ = create_service<nvblox_msgs::srv::FilePath>(
    "~/save_rates",
    std::bind(&NvbloxNode::saveRates, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, group_processing_);
  save_timings_service_ = create_service<nvblox_msgs::srv::FilePath>(
    "~/save_timings",
    std::bind(&NvbloxNode::saveTimings, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, group_processing_);
  send_esdf_and_gradient_service_ = create_service<nvblox_msgs::srv::EsdfAndGradients>(
    "~/get_esdf_and_gradient",
    std::bind(
      &NvbloxNode::getEsdfAndGradientService, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, group_processing_);
}

void NvbloxNode::setupTimers()
{
  RCLCPP_INFO_STREAM(get_logger(), "NvbloxNode::setupTimers()");

  queue_processing_timer_ =
    create_wall_timer(
    std::chrono::duration<double>(params_.tick_period_ms / 1000.0),
    std::bind(&NvbloxNode::tick, this), group_processing_);
}

// Functions internal to this file.
namespace
{

rclcpp::Time getTimestamp(const NitrosView & view)
{
  return rclcpp::Time(view.GetTimestampSeconds(), view.GetTimestampNanoseconds(), RCL_ROS_TIME);
}

}  // namespace

void NvbloxNode::depthImageCallback(const NitrosView & image_view)
{
  timing::Timer tick_timer("ros/depth_image_callback");
  timing::Rates::tick("ros/depth_image_callback");
  timing::Delays::tick(
    "ros/depth_image_callback",
    nvblox::Time(getTimestamp(image_view).nanoseconds()),
    nvblox::Time(now().nanoseconds()));
  printMessageArrivalStatistics(image_view, "Depth Statistics", &depth_frame_statistics_);

  pushMessageOntoQueue(
    "depth_queue",
    std::make_pair(std::make_shared<NitrosView>(image_view), image_view.GetFrameId()),
    &depth_image_queue_, &depth_queue_mutex_);
}

void NvbloxNode::colorImageCallback(const NitrosView & image_view)
{
  timing::Timer tick_timer("ros/color_image_callback");
  timing::Rates::tick("ros/color_image_callback");
  timing::Delays::tick(
    "ros/color_image_callback",
    nvblox::Time(getTimestamp(image_view).nanoseconds()),
    nvblox::Time(now().nanoseconds()));
  printMessageArrivalStatistics(image_view, "Color Statistics", &rgb_frame_statistics_);

  pushMessageOntoQueue(
    "color_queue",
    std::make_pair(std::make_shared<NitrosView>(image_view), image_view.GetFrameId()),
    &color_image_queue_, &color_queue_mutex_);
}

void NvbloxNode::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud)
{
  timing::Timer tick_timer("ros/pointcloud_callback");
  timing::Rates::tick("ros/pointcloud_callback");
  const rclcpp::Time pointcloud_stamp(pointcloud->header.stamp.sec,
    pointcloud->header.stamp.nanosec);
  timing::Delays::tick(
    "ros/pointcloud_callback", nvblox::Time(pointcloud_stamp.nanoseconds()),
    nvblox::Time(now().nanoseconds()));
  printMessageArrivalStatistics(
    *pointcloud, "Pointcloud Statistics",
    &pointcloud_frame_statistics_);

  pushMessageOntoQueue(
    "pointcloud_queue", pointcloud, &pointcloud_queue_,
    &pointcloud_queue_mutex_);
}

bool NvbloxNode::shouldProcess(
  const rclcpp::Time & time_now, const rclcpp::Time & time_last,
  const float desired_frequency_hz)
{
  if (desired_frequency_hz <= 0.f) {
    return false;
  }
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
  if (params_.use_depth) {
    processDepthQueue();
  }
  if (params_.use_color) {
    processColorQueue();
  }
  if (params_.use_lidar) {
    processPointcloudQueue();
  }

  // Decay
  if (const rclcpp::Time now = this->get_clock()->now();
    shouldProcess(now, decay_tsdf_last_time_, params_.decay_tsdf_rate_hz))
  {
    decayTsdf();
    decay_tsdf_last_time_ = now;
  }
  if (const rclcpp::Time now = this->get_clock()->now();
    (isUsingBothMappers(params_.mapping_type)) &&
    shouldProcess(
      now, decay_dynamic_occupancy_last_time_,
      params_.decay_dynamic_occupancy_rate_hz))
  {
    decayDynamicOccupancy();
    decay_dynamic_occupancy_last_time_ = now;
  }

  // Mapping
  if (const rclcpp::Time now = this->get_clock()->now(); shouldProcess(
      now, clear_map_outside_radius_last_time_, params_.clear_map_outside_radius_rate_hz))
  {
    clearMapOutsideOfRadiusOfLastKnownPose();
    clear_map_outside_radius_last_time_ = now;
  }

  // Output cost map
  if (const rclcpp::Time now = this->get_clock()->now();
    shouldProcess(now, update_esdf_last_time_, params_.update_esdf_rate_hz))
  {
    processEsdf();
    update_esdf_last_time_ = now;
  }

  // Visualization
  if (const rclcpp::Time now = this->get_clock()->now();
    shouldProcess(now, update_mesh_last_time_, params_.update_mesh_rate_hz))
  {
    processMesh();
    update_mesh_last_time_ = now;
  }

  if (const rclcpp::Time now = this->get_clock()->now();
    shouldProcess(now, publish_layer_last_time_, params_.publish_layer_rate_hz))
  {
    publishLayers();
    publish_layer_last_time_ = now;
  }

  // Restart the idle timer
  idle_timer_.Start();
}

void NvbloxNode::processDepthQueue()
{
  auto message_ready = [this](const NitrosViewPtrAndFrameId & msg) {
      return (msg.first != nullptr) && this->canTransform(msg.second, getTimestamp(*msg.first)) &&
             this->depth_camera_cache_.hasCameraForFrameId(msg.second);
    };

  processMessageQueue<NitrosViewPtrAndFrameId>(
    &depth_image_queue_,    // NOLINT
    &depth_queue_mutex_,    // NOLINT
    message_ready,          // NOLINT
    std::bind(&NvbloxNode::processDepthImage, this, std::placeholders::_1));
}

void NvbloxNode::processColorQueue()
{
  auto message_ready = [this](const NitrosViewPtrAndFrameId & msg) {
      return this->canTransform(msg.second, getTimestamp(*msg.first)) &&
             this->color_camera_cache_.hasCameraForFrameId(msg.second);
    };

  processMessageQueue<NitrosViewPtrAndFrameId>(
    &color_image_queue_,    // NOLINT
    &color_queue_mutex_,    // NOLINT
    message_ready,          // NOLINT
    std::bind(&NvbloxNode::processColorImage, this, std::placeholders::_1));
}

void NvbloxNode::processPointcloudQueue()
{
  using PointcloudMsg = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
  auto message_ready = [this](const PointcloudMsg & msg) {
      return this->canTransform(msg->header.frame_id, msg->header.stamp);
    };
  processMessageQueue<PointcloudMsg>(
    &pointcloud_queue_,          // NOLINT
    &pointcloud_queue_mutex_,    // NOLINT
    message_ready,               // NOLINT
    std::bind(&NvbloxNode::processLidarPointcloud, this, std::placeholders::_1));
}

void NvbloxNode::processEsdf()
{
  const rclcpp::Time timestamp = get_clock()->now();
  timing::Timer ros_esdf_timer("ros/esdf");
  timing::Rates::tick("ros/update_esdf");

  timing::Timer esdf_integration_timer("ros/esdf/integrate");
  multi_mapper_->updateEsdf();
  if (newest_integrated_depth_time_ > rclcpp::Time(0, 0, RCL_ROS_TIME)) {
    timing::Delays::tick(
      "ros/esdf_integration",
      nvblox::Time(newest_integrated_depth_time_.nanoseconds()),
      nvblox::Time(now().nanoseconds()));
  }
  esdf_integration_timer.Stop();

  timing::Timer esdf_output_timer("ros/esdf/output");

  sliceAndPublishEsdf(
    "static", static_mapper_, static_esdf_pointcloud_publisher_,
    static_map_slice_publisher_);

  if (isUsingBothMappers(params_.mapping_type)) {
    sliceAndPublishEsdf(
      "dynamic", dynamic_mapper_, dynamic_esdf_pointcloud_publisher_,
      dynamic_map_slice_publisher_);
    sliceAndPublishEsdf(
      "combined_dynamic", static_mapper_, combined_esdf_pointcloud_publisher_,
      combined_map_slice_publisher_, dynamic_mapper_.get());
  }

  // Also publish the slice bounds (showing esdf max/min 2d height)
  if (slice_bounds_publisher_->get_subscription_count() > 0) {
    // The frame to which the slice limits visualization is attached.
    // We get the transform from the plane-body (PB) frame, to the scene (S).
    Transform T_S_PB;
    if (transformer_.lookupTransformToGlobalFrame(
        params_.slice_visualization_attachment_frame_id,
        rclcpp::Time(0), &T_S_PB))
    {
      // Get and publish the planes representing the slice bounds in z.
      const visualization_msgs::msg::Marker marker_bottom = sliceLimitsToMarker(
        T_S_PB, params_.slice_visualization_side_length, timestamp, params_.global_frame.get(),
        static_mapper_->esdf_slice_min_height(), SliceLimitMarkerType::kBottomSliceLimit);
      const visualization_msgs::msg::Marker marker_top = sliceLimitsToMarker(
        T_S_PB, params_.slice_visualization_side_length, timestamp, params_.global_frame.get(),
        static_mapper_->esdf_slice_max_height(), SliceLimitMarkerType::kTopSliceLimit);
      slice_bounds_publisher_->publish(marker_bottom);
      slice_bounds_publisher_->publish(marker_top);
    } else {
      RCLCPP_INFO_STREAM_THROTTLE(
        get_logger(), *get_clock(), kTimeBetweenDebugMessagesMs,
        "Tried to publish slice bounds but couldn't look up frame: "
          << params_.slice_visualization_attachment_frame_id.get());
    }
  }
}

void NvbloxNode::sliceAndPublishEsdf(
  const std::string & name, const std::shared_ptr<Mapper> & mapper,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pointcloud_publisher,
  const rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr & slice_publisher,
  const Mapper * mapper_2)
{
  // If anyone wants a slice
  if (pointcloud_publisher->get_subscription_count() > 0 ||
    (params_.publish_esdf_distance_slice && slice_publisher->get_subscription_count() > 0))
  {
    // Get the slice as an image
    timing::Timer slicing_timer("ros/" + name + "/esdf/output/compute");
    AxisAlignedBoundingBox aabb;
    Image<float> map_slice_image(MemoryType::kDevice);
    if (mapper_2 != nullptr) {
      esdf_slice_converter_.sliceLayersToCombinedDistanceImage(
        mapper->esdf_layer(), mapper_2->esdf_layer(), mapper->esdf_slice_height(),
        mapper_2->esdf_slice_height(), &map_slice_image, &aabb);
    } else {
      esdf_slice_converter_.sliceLayerToDistanceImage(
        mapper->esdf_layer(), mapper->esdf_slice_height(), &map_slice_image, &aabb);
    }
    slicing_timer.Stop();

    // Publish a pointcloud of the slice image for visualization in RVIZ
    if (pointcloud_publisher->get_subscription_count() > 0) {
      timing::Timer pointcloud_msg_timer("ros/" + name + "/esdf/output/pointcloud");
      sensor_msgs::msg::PointCloud2 pointcloud_msg;
      esdf_slice_converter_.pointcloudMsgFromSliceImage(
        map_slice_image, aabb, mapper->esdf_slice_height(), mapper->esdf_layer().voxel_size(),
        &pointcloud_msg);
      pointcloud_msg.header.frame_id = params_.global_frame.get();
      pointcloud_msg.header.stamp = get_clock()->now();
      pointcloud_publisher->publish(pointcloud_msg);
    }

    // Publish the distance map slice (costmap for nav2).
    if (params_.publish_esdf_distance_slice && slice_publisher->get_subscription_count() > 0) {
      timing::Timer slice_msg_timer("ros/" + name + "/esdf/output/slice");
      nvblox_msgs::msg::DistanceMapSlice map_slice_msg;
      esdf_slice_converter_.distanceMapSliceMsgFromSliceImage(
        map_slice_image, aabb, mapper->esdf_slice_height(), mapper->voxel_size_m(),
        &map_slice_msg);
      map_slice_msg.header.frame_id = params_.global_frame.get();
      map_slice_msg.header.stamp = get_clock()->now();
      slice_publisher->publish(map_slice_msg);
    }
  }
}

void NvbloxNode::processMesh()
{
  const rclcpp::Time timestamp = get_clock()->now();
  timing::Timer ros_mesh_timer("ros/mesh");
  timing::Timer mesh_update_timer("ros/mesh/update");

  Transform T_L_C;
  if (!transformer_.lookupTransformToGlobalFrame(
      params_.map_clearing_frame_id, rclcpp::Time(0),
      &T_L_C))
  {
    RCLCPP_INFO_STREAM_THROTTLE(
      get_logger(), *get_clock(), kTimeBetweenDebugMessagesMs,
      "Lookup transform failed for frame "
        << params_.map_clearing_frame_id.get()
        << ". Mesh not published");
    return;
  }
  bool serialize_full_mesh = false;
  size_t new_subscriber_count = mesh_publisher_->get_subscription_count();
  // In case we have new subscribers, publish the ENTIRE map once.
  nvblox_msgs::msg::Mesh mesh_msg;
  if (new_subscriber_count > mesh_subscriber_count_) {
    RCLCPP_INFO(get_logger(), "Got a new subscriber, sending entire map.");
    serialize_full_mesh = true;
  }
  mesh_subscriber_count_ = new_subscriber_count;

  // Update and serialize the mesh
  std::shared_ptr<const SerializedMesh> serialized_mesh =
    multi_mapper_->updateMesh(T_L_C, serialize_full_mesh);
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "Num of streamed mesh blocks: " << serialized_mesh->block_indices.size());
  mesh_update_timer.Stop();

  // Publish the mesh updates.
  timing::Timer mesh_output_timer("ros/mesh/output");
  if (mesh_subscriber_count_ > 0) {
    // Delete mesh blocks from the visualization if they were deallocated in the mesh layer.
    // In the case that some mesh blocks have been re-added after deallocation,
    // remove them from the list to delete.
    const std::vector<Index3D> & mesh_block_not_to_delete = serialized_mesh->block_indices;
    const std::vector<Index3D> mesh_blocks_to_delete =
      static_mapper_->getClearedMeshBlocks(mesh_block_not_to_delete);
    if (!mesh_blocks_to_delete.empty()) {
      nvblox_msgs::msg::Mesh deleted_mesh_msg;
      conversions::meshMessageFromBlocksToDelete(
        mesh_blocks_to_delete, timestamp, params_.global_frame.get(),
        static_mapper_->mesh_layer().block_size(), &deleted_mesh_msg);
      mesh_publisher_->publish(deleted_mesh_msg);
    }

    // Publish mesh blocks from serialized mesh
    if (!serialized_mesh->block_indices.empty()) {
      const bool resend_full_mesh = serialize_full_mesh;
      conversions::meshMessageFromSerializedMesh(
        serialized_mesh, timestamp, params_.global_frame.get(),
        static_mapper_->mesh_layer().block_size(), resend_full_mesh, &mesh_msg);
      mesh_publisher_->publish(mesh_msg);
      timing::Rates::tick("ros/mesh");
    }
  }

  // optionally publish the markers.
  if (params_.enable_mesh_markers && mesh_marker_publisher_->get_subscription_count() > 0) {
    visualization_msgs::msg::MarkerArray marker_msg;
    conversions::markerMessageFromSerializedMesh(
      serialized_mesh, params_.global_frame.get(),
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
  static_mapper_->decayTsdf();
}

bool NvbloxNode::canTransform(const std::string & frame_id, const rclcpp::Time & timestamp)
{
  Transform T_L_C;
  return transformer_.lookupTransformToGlobalFrame(frame_id, timestamp, &T_L_C);
}

bool NvbloxNode::processDepthImage(const NitrosViewPtrAndFrameId & view_and_frameid)
{
  timing::Timer ros_depth_timer("ros/depth");
  timing::Timer transform_timer("ros/depth/transform");

  const NitrosViewPtr nitros_view = view_and_frameid.first;
  const std::string depth_frame = view_and_frameid.second;

  // Check if the message is too soon, and if it is discard it
  const rclcpp::Time depth_image_timestamp = getTimestamp(*view_and_frameid.first);

  if (integrate_depth_last_times_.find(depth_frame) == integrate_depth_last_times_.end()) {
    integrate_depth_last_times_[depth_frame] = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  const rclcpp::Time last_depth_image_timestamp = integrate_depth_last_times_[depth_frame];
  if (!shouldProcess(
      depth_image_timestamp, last_depth_image_timestamp,
      params_.integrate_depth_rate_hz))
  {
    rclcpp::Duration time_since_last = depth_image_timestamp - last_depth_image_timestamp;
    RCLCPP_WARN_STREAM(
      get_logger(), "Discarding depth frame: "
        << depth_frame
        << ". time_since_last: " << time_since_last.seconds()
        << ". Desired: " << 1.0 / params_.integrate_depth_rate_hz);

    // To discard we indicate that the image was processed, without actually integrating it.
    return true;
  }
  // Get the timestamp
  constexpr int64_t kNanoSecondsToMilliSeconds = 1e6;
  nvblox::Time update_time_ms(depth_image_timestamp.nanoseconds() / kNanoSecondsToMilliSeconds);

  // Get the TF for this image.
  Transform T_L_C;
  if (!transformer_.lookupTransformToGlobalFrame(depth_frame, depth_image_timestamp, &T_L_C)) {
    return false;
  }
  transform_timer.Stop();
  timing::Timer conversions_timer("ros/depth/conversions");

  // Lookup the camera from the cache.
  auto maybe_camera = depth_camera_cache_.getCameraForFrameId(depth_frame);
  CHECK(maybe_camera.has_value());

  // Convert the depth image.
  if (!conversions::depthImageFromNitrosViewAsync(
      *nitros_view, &depth_image_, get_logger(),
      cuda_stream_))
  {
    RCLCPP_ERROR(get_logger(), "Failed to transform depth image.");
    return false;
  }
  conversions_timer.Stop();
  // Do the actual depth integration
  timing::Timer integration_timer("ros/depth/integrate");
  timing::Rates::tick("ros/depth");
  multi_mapper_->integrateDepth(depth_image_, T_L_C, maybe_camera.value(), update_time_ms);
  timing::Delays::tick(
    "ros/depth_image_integration",
    nvblox::Time(depth_image_timestamp.nanoseconds()),
    nvblox::Time(now().nanoseconds()));
  integrate_depth_last_times_[depth_frame] = depth_image_timestamp;
  // Update the most recent depth time
  newest_integrated_depth_time_ = std::max(depth_image_timestamp, newest_integrated_depth_time_);
  integration_timer.Stop();
  // Optional publishing - Freespace + Dynamics
  if (isDynamicMapping(params_.mapping_type)) {
    // Process the freespace layer
    // TODO(dtingdahl) Move this into the publishLayers() function so we publish visualization
    // messages at the same place (and separated from processing)
    timing::Timer dynamic_publishing_timer("ros/depth/output/dynamics");
    publishDynamics(depth_frame);
    dynamic_publishing_timer.Stop();
  }
  // Publish back projected depth image for debugging
  timing::Timer back_projected_depth_timer("ros/depth/output/back_projected_depth");
  if (back_projected_depth_publisher_->get_subscription_count() > 0) {
    // Only back project and publish every n-th depth image
    if (back_projection_idx_ % params_.back_projection_subsampling == 0) {
      publishBackProjectedDepth(maybe_camera.value(), T_L_C);
    }
    back_projection_idx_++;
  }
  back_projected_depth_timer.Stop();
  return true;
}

void NvbloxNode::publishDynamics(const std::string & camera_frame_id)
{
  // Publish the dynamic pointcloud
  if (dynamic_points_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 dynamic_points_msg;
    const Pointcloud & dynamic_points = multi_mapper_->getLastDynamicPointcloud();
    pointcloud_converter_.pointcloudMsgFromPointcloud(dynamic_points, &dynamic_points_msg);
    dynamic_points_msg.header.frame_id = params_.global_frame.get();
    dynamic_points_msg.header.stamp = get_clock()->now();
    dynamic_points_publisher_->publish(dynamic_points_msg);
  }

  // Publish the dynamic overlay
  if (dynamic_depth_frame_overlay_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::Image img_msg;
    const ColorImage & dynamic_overlay = multi_mapper_->getLastDynamicFrameMaskOverlay();
    conversions::imageMessageFromColorImage(
      dynamic_overlay, camera_frame_id, &img_msg,
      cuda_stream_);
    dynamic_depth_frame_overlay_publisher_->publish(img_msg);
  }
}

void NvbloxNode::publishBackProjectedDepth(const Camera & camera, const Transform & T_L_C)
{
  // Get the pointcloud from the depth image
  image_back_projector_.backProjectOnGPU(
    depth_image_, camera, &pointcloud_C_device_,
    params_.max_back_projection_distance);
  transformPointcloudOnGPU(T_L_C, pointcloud_C_device_, &pointcloud_L_device_);

  // Send the message
  sensor_msgs::msg::PointCloud2 back_projected_depth_msg;
  pointcloud_converter_.pointcloudMsgFromPointcloud(
    pointcloud_L_device_,
    &back_projected_depth_msg);
  back_projected_depth_msg.header.frame_id = params_.global_frame.get();
  back_projected_depth_msg.header.stamp = get_clock()->now();
  back_projected_depth_publisher_->publish(back_projected_depth_msg);
}

bool NvbloxNode::processColorImage(const NitrosViewPtrAndFrameId & view_and_frameid)
{
  timing::Timer ros_color_timer("ros/color");
  timing::Timer transform_timer("ros/color/transform");

  const NitrosViewPtr nitros_view = view_and_frameid.first;
  const std::string color_frame = view_and_frameid.second;

  // Check if the message is too soon, and if it is discard it
  const rclcpp::Time color_image_timestamp = getTimestamp(*view_and_frameid.first);
  if (integrate_color_last_times_.find(color_frame) == integrate_color_last_times_.end()) {
    integrate_color_last_times_[color_frame] = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  const rclcpp::Time last_color_image_timestamp = integrate_color_last_times_[color_frame];
  if (!shouldProcess(
      color_image_timestamp, last_color_image_timestamp,
      params_.integrate_color_rate_hz))
  {
    // To discard we indicate that the image was processed, without actually integrating it.
    return true;
  }

  // Get the TF for this image.
  Transform T_L_C;

  if (!transformer_.lookupTransformToGlobalFrame(color_frame, color_image_timestamp, &T_L_C)) {
    return false;
  }

  transform_timer.Stop();

  timing::Timer color_convert_timer("ros/color/conversion");

  // Lookup the camera from the cache.
  auto maybe_camera = color_camera_cache_.getCameraForFrameId(color_frame);
  CHECK(maybe_camera.has_value());

  // Convert the color image.
  if (!conversions::colorImageFromNitrosViewAsync(
      *nitros_view, &color_image_, get_logger(),
      cuda_stream_))
  {
    RCLCPP_ERROR(get_logger(), "Failed to transform color image.");
    return false;
  }
  color_convert_timer.Stop();

  // Integrate.
  timing::Timer color_integrate_timer("ros/color/integrate");
  timing::Rates::tick("ros/color");
  multi_mapper_->integrateColor(color_image_, T_L_C, maybe_camera.value());
  timing::Delays::tick(
    "ros/color_image_integration",
    nvblox::Time(color_image_timestamp.nanoseconds()),
    nvblox::Time(now().nanoseconds()));
  integrate_color_last_times_[color_frame] = color_image_timestamp;
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
  if (!shouldProcess(
      pointcloud_timestamp, integrate_lidar_last_time_,
      params_.integrate_lidar_rate_hz))
  {
    // To discard we indicate that the image was processed, without actually integrating it.
    return true;
  }

  // Get the TF for this image.
  const std::string target_frame = pointcloud_ptr->header.frame_id;
  Transform T_L_C;

  if (!transformer_.lookupTransformToGlobalFrame(
      target_frame, pointcloud_ptr->header.stamp,
      &T_L_C))
  {
    return false;
  }

  transform_timer.Stop();

  // Create the LiDAR model. The method of creation depends on whether we have
  // equal FoV above and below 0 elevation. This is specified through a param.
  Lidar lidar =
    (params_.use_non_equal_vertical_fov_lidar_params) ?
    Lidar(
    params_.lidar_width, params_.lidar_height,
    params_.min_angle_below_zero_elevation_rad,
    params_.max_angle_above_zero_elevation_rad) :
    Lidar(params_.lidar_width, params_.lidar_height, params_.lidar_vertical_fov_rad);

  // We check that the pointcloud is consistent with this LiDAR model
  // NOTE(alexmillane): If the check fails we return true which indicates that
  // this pointcloud can be removed from the queue even though it wasn't
  // integrated (because the intrisics model is messed up).
  // NOTE(alexmillane): Note that internally we cache checks, so each LiDAR
  // intrisics model is only tested against a single pointcloud. This is because
  // the check is expensive to perform.
  if (!pointcloud_converter_.checkLidarPointcloud(pointcloud_ptr, lidar)) {
    RCLCPP_ERROR_ONCE(
      get_logger(), "LiDAR intrinsics are inconsistent with the received "
      "pointcloud. Failing integration.");
    return true;
  }

  timing::Timer lidar_conversion_timer("ros/lidar/conversion");
  pointcloud_converter_.depthImageFromPointcloudGPU(pointcloud_ptr, lidar, &pointcloud_image_);
  lidar_conversion_timer.Stop();

  timing::Timer lidar_integration_timer("ros/lidar/integration");
  static_mapper_->integrateLidarDepth(pointcloud_image_, T_L_C, lidar);
  timing::Delays::tick(
    "ros/pointcloud_integration",
    nvblox::Time(pointcloud_timestamp.nanoseconds()),
    nvblox::Time(now().nanoseconds()));
  newest_integrated_depth_time_ = std::max(pointcloud_timestamp, newest_integrated_depth_time_);
  integrate_lidar_last_time_ = pointcloud_timestamp;
  lidar_integration_timer.Stop();

  return true;
}

void NvbloxNode::publishLayers()
{
  timing::Timer publish_layer_timer("ros/publish_layer");

  Transform T_L_C;
  if (!transformer_.lookupTransformToGlobalFrame(
      params_.map_clearing_frame_id, rclcpp::Time(0),
      &T_L_C))
  {
    RCLCPP_INFO_STREAM_THROTTLE(
      get_logger(), *get_clock(), kTimeBetweenDebugMessagesMs,
      "Lookup transform failed for frame "
        << params_.map_clearing_frame_id.get()
        << ". Layer pointclouds not published");
    return;
  }

  visualization_msgs::msg::Marker layer_msg;
  layer_msg.header.frame_id = params_.global_frame.get();
  layer_msg.header.stamp = get_clock()->now();

  // TSDF
  if (tsdf_layer_publisher_ && tsdf_layer_publisher_->get_subscription_count() > 0) {
    layer_converter_.markerMsgFromTsdfLayer(
      static_mapper_->tsdf_layer(), params_.layer_visualization_max_tsdf_distance_m,
      params_.layer_visualization_min_tsdf_weight, params_.layer_visualization_exclusion_height_m,
      params_.layer_visualization_exclusion_radius_m, T_L_C.translation(), &layer_msg,
      cuda_stream_);

    tsdf_layer_publisher_->publish(layer_msg);
  }

  // Static occupancy
  if (occupancy_layer_publisher_ && occupancy_layer_publisher_->get_subscription_count() > 0) {
    layer_converter_.markerMsgFromOccupancyLayer(
      static_mapper_->occupancy_layer(),
      params_.layer_visualization_exclusion_height_m,
      params_.layer_visualization_exclusion_radius_m,
      T_L_C.translation(), &layer_msg, cuda_stream_);

    occupancy_layer_publisher_->publish(layer_msg);
  }

  // Color
  if (color_layer_publisher_ && color_layer_publisher_->get_subscription_count() > 0) {
    if (params_.esdf_mode == EsdfMode::k3D) {
      layer_converter_.markerMsgFromColorLayer(
        static_mapper_->esdf_layer(),
        static_mapper_->color_layer(),
        params_.layer_visualization_undo_gamma_correction,
        params_.layer_visualization_exclusion_height_m,
        params_.layer_visualization_exclusion_radius_m,
        T_L_C.translation(), &layer_msg, cuda_stream_);

    } else {
      layer_converter_.markerMsgFromColorLayer(
        static_mapper_->tsdf_layer(),
        static_mapper_->color_layer(),
        params_.layer_visualization_undo_gamma_correction,
        params_.layer_visualization_max_tsdf_distance_m,
        params_.layer_visualization_min_tsdf_weight,
        params_.layer_visualization_exclusion_height_m,
        params_.layer_visualization_exclusion_radius_m,
        T_L_C.translation(), &layer_msg, cuda_stream_);
    }
    color_layer_publisher_->publish(layer_msg);
  }

  // Freespace
  if (freespace_layer_publisher_ && freespace_layer_publisher_->get_subscription_count() > 0) {
    layer_converter_.markerMsgFromFreespaceLayer(
      static_mapper_->tsdf_layer(), static_mapper_->freespace_layer(),
      params_.layer_visualization_max_tsdf_distance_m,
      params_.layer_visualization_min_tsdf_weight, params_.layer_visualization_exclusion_height_m,
      params_.layer_visualization_exclusion_radius_m, T_L_C.translation(), &layer_msg,
      cuda_stream_);

    freespace_layer_publisher_->publish(layer_msg);
  }

  // Dynamic occupancy
  if (dynamic_occupancy_layer_publisher_ &&
    dynamic_occupancy_layer_publisher_->get_subscription_count() > 0)
  {
    layer_converter_.markerMsgFromOccupancyLayer(
      dynamic_mapper_->occupancy_layer(),
      params_.layer_visualization_exclusion_height_m,
      params_.layer_visualization_exclusion_radius_m,
      T_L_C.translation(), &layer_msg, cuda_stream_);

    dynamic_occupancy_layer_publisher_->publish(layer_msg);
  }

  publish_layer_timer.Stop();
}

void NvbloxNode::clearMapOutsideOfRadiusOfLastKnownPose()
{
  if (params_.map_clearing_radius_m > 0.0f) {
    timing::Timer("ros/clear_outside_radius");
    Transform T_L_MC;  // MC = map clearing frame
    if (transformer_.lookupTransformToGlobalFrame(
        params_.map_clearing_frame_id, rclcpp::Time(0),
        &T_L_MC))
    {
      static_mapper_->clearOutsideRadius(T_L_MC.translation(), params_.map_clearing_radius_m);
    } else {
      RCLCPP_INFO_STREAM_THROTTLE(
        get_logger(), *get_clock(), kTimeBetweenDebugMessagesMs,
        "Tried to clear map outside of radius but couldn't look up frame: "
          << params_.map_clearing_frame_id.get());
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
    success = io::outputMeshLayerToPly(static_mapper_->mesh_layer(), request->file_path);
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
    if (isDynamicMapping(params_.mapping_type)) {
      success &= io::outputVoxelLayerToPly(
        static_mapper_->freespace_layer(),
        request->file_path + "/ros2_freespace.ply");
    }
  }
  if (success) {
    RCLCPP_INFO_STREAM(get_logger(), "Output PLY file(s) to " << request->file_path);
    response->success = true;
  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Failed to write PLY file(s) to " << request->file_path);
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
    RCLCPP_WARN_STREAM(get_logger(), "Failed to load map file from " << filename);
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
    RCLCPP_WARN_STREAM(get_logger(), "The rates file should be a text file!!!");
  }
  if (success) {
    RCLCPP_INFO_STREAM(get_logger(), "Output rates to " << request->file_path);
    response->success = true;
  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Failed to write rates to " << request->file_path);
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
    RCLCPP_WARN_STREAM(get_logger(), "The timings file should be a text file!!!");
  }
  if (success) {
    RCLCPP_INFO_STREAM(get_logger(), "Output timings to " << request->file_path);
    response->success = true;
  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Failed to write timings to " << request->file_path);
    response->success = false;
  }
}

void NvbloxNode::getEsdfAndGradientService(
  const std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Request> request,
  std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Response> response)
{
  RCLCPP_INFO(
    get_logger(),
    "Received request for ESDF with:\n\aabb_min_m: [%0.2f, %0.2f, %0.2f], "
    "\n\aabb_size_m: [%0.2f, %0.2f, %0.2f]\n",
    request->aabb_min_m.x, request->aabb_min_m.y, request->aabb_min_m.z,
    request->aabb_size_m.x, request->aabb_size_m.y, request->aabb_size_m.z);
  if (params_.esdf_mode != EsdfMode::k3D) {
    RCLCPP_FATAL_STREAM(
      get_logger(),
      "The ESDF service is only intended for mapping with 3D ESDFs. You're in 2D mode."
      "To use this function set esdf_mode: 1. Exiting.");
    exit(1);
  }

  // Construct the bounding box.
  const Vector3f aabb_min_m(request->aabb_min_m.x, request->aabb_min_m.y, request->aabb_min_m.z);
  const Vector3f aabb_size_m(request->aabb_size_m.x, request->aabb_size_m.y,
    request->aabb_size_m.z);
  AxisAlignedBoundingBox aabb(aabb_min_m, aabb_min_m + aabb_size_m);

  // Convert the layer to a message.
  response->voxel_size.data = static_mapper_->esdf_layer().voxel_size();
  response->esdf_and_gradients = esdf_and_gradients_converter_.esdfInAabbToMultiArrayMsg(
    static_mapper_->esdf_layer(), aabb, params_.esdf_and_gradients_unobserved_value,
    cuda_stream_);
}

}  // namespace nvblox

// Register the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvblox::NvbloxNode)
