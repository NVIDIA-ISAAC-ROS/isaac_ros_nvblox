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
#include "nvblox_ros/conversions/occupancy_conversions.hpp"
#include "nvblox_ros/utils.hpp"
#include "nvblox_ros/visualization.hpp"

namespace nvblox
{

// Functions internal to this file.
namespace
{

// This Visitor is a pattern which is used to inherit the call operator from several other classes.
// Usually this is used, including in this file to inherit from several lambdas.
// For more see: https://medium.com/@nerudaj/std-visit-is-awesome-heres-why-f183f6437932
template<class ... Ts>
struct Visitor : Ts ... { using Ts::operator() ...; };

template<class ... Ts>
Visitor(Ts ...)->Visitor<Ts...>;

rclcpp::Time getTimestamp(const NitrosView & view)
{
  return rclcpp::Time(view.GetTimestampSeconds(), view.GetTimestampNanoseconds(), RCL_ROS_TIME);
}

}  // namespace

NvbloxNode::NvbloxNode(
  const rclcpp::NodeOptions & options, const std::string & node_name,
  std::shared_ptr<CudaStream> cuda_stream)
: Node(node_name, options), transformer_(this), cuda_stream_(cuda_stream)
{
  // Passing the ROS clock to the rates singleton.
  // This ensures that the rates are relative to ROS and not to system time.
  // NOTE(remos): For timing::Timer we rely on system time to not make our
  // measurements dependent on e.g. simulation time speed.
  auto getROSTimestampFunctor = [this]() -> uint64_t {
      return this->get_clock()->now().nanoseconds();
    };
  timing::Rates::setGetTimestampFunctor(getROSTimestampFunctor);

  // Get parameters first (stuff below depends on parameters)
  initializeNvbloxNodeParams(this, &params_, &parameter_tree_);
  CHECK_LE(params_.num_cameras, kMaxNumCameras);

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

  RCLCPP_INFO_STREAM(
    get_logger(), "Started up nvblox node in frame " << params_.global_frame.get()
                                                     << " and voxel size "
                                                     << params_.voxel_size);

  // Check if a valid mapping typ was selected
  RCLCPP_INFO_STREAM(get_logger(), "Mapping type: " << toString(params_.mapping_type));

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
  if (params_.after_shutdown_map_save_path.get().size() > 0) {
    // Save map after shutdown

    // Grab a slice
    AxisAlignedBoundingBox aabb;
    Image<float> map_slice_image(MemoryType::kDevice);
    esdf_slice_converter_.sliceLayerToDistanceImage(
      static_mapper_->esdf_layer(), static_mapper_->esdf_integrator().esdf_slice_height(),
      params_.distance_map_unknown_value_optimistic, &map_slice_image, &aabb);

    const size_t width = map_slice_image.cols();
    const size_t height = map_slice_image.rows();

    if (width == 0 || height == 0) {
      RCLCPP_INFO_STREAM(get_logger(), "No map to save, skipping map save.");
    } else {
      std::vector<signed char> occupancy_grid;
      occupancy_grid.resize(width * height, kOccupancyGridUnknownValue);

      // Convert from ESDF to occupancy grid
      esdf_slice_converter_.occupancyGridFromSliceImage(
        map_slice_image, occupancy_grid.data(), params_.distance_map_unknown_value_optimistic);

      constexpr float kFreeThreshold = 0.25;
      constexpr float kOccupiedThreshold = 0.65;

      std::string png_path = params_.after_shutdown_map_save_path.get() + ".png";
      nvblox::conversions::saveOccupancyGridAsPng(
        png_path, kFreeThreshold, kOccupiedThreshold,
        height, width, occupancy_grid);
      RCLCPP_INFO_STREAM(get_logger(), "Writing occupancy map to: " << png_path);

      const int file_name_index = png_path.find_last_of("/\\");
      std::string image_name = png_path.substr(file_name_index + 1);
      std::string yaml_path = params_.after_shutdown_map_save_path.get() + ".yaml";
      nvblox::conversions::saveOccupancyGridYaml(
        yaml_path, image_name, static_mapper_->esdf_layer().voxel_size(), aabb.min().x(),
        aabb.min().y(), kFreeThreshold, kOccupiedThreshold);
      RCLCPP_INFO_STREAM(get_logger(), "Writing occupancy map yaml meta data to: " << yaml_path);
    }
  }

  // Having the destructor destroying NITROS types may fail if the process-wide GXF/NITROS context
  // has been released elsewhere. This is out of control of this node. To avoid crashes, we refrain
  // from deleting them and instead relay on the OS for memory cleanup. Note that this will lead to
  // memory leaks if several classes are instantiated in the same process.
  depth_image_queue_.release();
  color_image_queue_.release();

  RCLCPP_INFO_STREAM(get_logger(), "Timing statistics: \n" << nvblox::timing::Timing::Print());
  RCLCPP_INFO_STREAM(get_logger(), "Rates statistics: \n" << nvblox::timing::Rates::Print());
  RCLCPP_INFO_STREAM(get_logger(), "Delay statistics: \n" << nvblox::timing::Delays::Print());
}

void NvbloxNode::initializeMultiMapper()
{
  // Create the multi mapper
  // NOTE(remos): Mesh integration is not implemented for occupancy layers.
  multi_mapper_ =
    std::make_shared<MultiMapper>(
    params_.voxel_size, params_.mapping_type, params_.esdf_mode,
    MemoryType::kDevice, cuda_stream_);

  // Get the mapper parameters
  const std::string static_mapper_name = "static_mapper";
  const std::string dynamic_mapper_name = "dynamic_mapper";
  declareMapperParameters(static_mapper_name, this);
  declareMapperParameters(dynamic_mapper_name, this);
  declareMultiMapperParameters(this);
  MapperParams static_mapper_params = getMapperParamsFromROS(static_mapper_name, this);
  MapperParams dynamic_mapper_params = getMapperParamsFromROS(dynamic_mapper_name, this);
  MultiMapperParams multi_mapper_params = getMultiMapperParamsFromROS(this);

  // Set the mapper parameters
  multi_mapper_->setMapperParams(static_mapper_params, dynamic_mapper_params);
  multi_mapper_->setMultiMapperParams(multi_mapper_params);

  // Get direct handles to the underlying mappers
  // NOTE(remos): Ideally, everything would be handled by the multi mapper
  //              and these handles wouldn't be needed.
  static_mapper_ = multi_mapper_.get()->background_mapper();
  dynamic_mapper_ = multi_mapper_.get()->foreground_mapper();
}

void NvbloxNode::subscribeToTopics()
{
  RCLCPP_INFO_STREAM(get_logger(), "NvbloxNode::subscribeToTopics()");

  // Create subscriber queues.
  depth_image_queue_ = std::make_unique<std::list<ImageTypeVariant>>();
  color_image_queue_ = std::make_unique<std::list<ImageTypeVariant>>();
  pointcloud_queue_ = std::make_unique<std::list<sensor_msgs::msg::PointCloud2::ConstSharedPtr>>();
  esdf_service_queue_ = std::make_unique<std::list<EsdfServiceQueuedType>>();
  file_path_service_queue_ = std::make_unique<std::list<FilePathServiceQueuedType>>();

  constexpr int kQueueSize = 10;

  if (!params_.use_depth && !params_.use_lidar) {
    RCLCPP_WARN(
      get_logger(), "Nvblox is running without depth or lidar input, the cost maps and"
      " reconstructions will not update");
  }

  size_t camera_index = 0;

  // Settings for QoS.
  const rclcpp::QoS input_qos =
    isaac_ros::common::AddQosParameter(*this, kDefaultInputQos_, "input_qos");
  std::string input_qos_str = kDefaultInputQos_;
  get_parameter("input_qos", input_qos_str);
  const rmw_qos_profile_t input_qos_profile = input_qos.get_rmw_qos_profile();
  RCLCPP_INFO_STREAM(get_logger(), "Subscribing input topics with QoS: " << input_qos_str);

  // We subscribe to the masks if we enable segmentation
  // We call depth/color+mask queue if segmentation is enabled.
  // Else we fill the depth/color queue. This is important as both
  // the call back functions are always called by the tick function.
  // If segmentation is enabled we would want the depth/color queue to be empty.
  if (params_.use_depth) {
    for (int i = camera_index; i < params_.num_cameras; ++i) {
      const std::string base_name_depth(kDepthTopicBaseNames[i]);
      depth_camera_info_subs_.emplace_back(
        std::make_shared<::message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
          this, base_name_depth + "/camera_info", input_qos_profile));

      depth_image_subs_.emplace_back(
        std::make_shared<nvidia::isaac_ros::nitros::message_filters::Subscriber<NitrosView>>(
          this, base_name_depth + "/image",
          input_qos_profile));
      if (params_.use_segmentation) {
        const std::string base_name_seg_depth(kSegTopicBaseNames[i]);
        segmentation_camera_info_subs_.emplace_back(
          std::make_shared<::message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
            this, base_name_seg_depth + "/camera_info", input_qos_profile));

        segmentation_image_subs_.emplace_back(
          std::make_shared<nvidia::isaac_ros::nitros::message_filters::Subscriber<NitrosView>>(
            this, base_name_seg_depth + "/image",
            input_qos_profile));
        // Sync depth and segmentation images with their camera infos
        timesync_depth_mask_.emplace_back(
          std::make_shared<image_mask_approx_sync>(
            image_mask_approx_sync(kQueueSize), *depth_image_subs_.back(),
            *depth_camera_info_subs_.back(),
            *segmentation_image_subs_.back(), *segmentation_camera_info_subs_.back()));

        timesync_depth_mask_.back()->registerCallback(
          std::bind(
            &NvbloxNode::depthPlusMaskImageCallback, this,
            std::placeholders::_1, std::placeholders::_2,
            std::placeholders::_3, std::placeholders::_4));
      } else {
        // Sync only depth image with its camera info
        timesync_depth_.emplace_back(
          std::make_shared<image_exact_sync>(
            image_exact_sync(kQueueSize), *depth_image_subs_.back(),
            *depth_camera_info_subs_.back()));

        timesync_depth_.back()->registerCallback(
          std::bind(
            &NvbloxNode::depthImageCallback, this,
            std::placeholders::_1, std::placeholders::_2));
      }
    }
  }

  if (params_.use_color) {
    for (int i = camera_index; i < params_.num_cameras; ++i) {
      const std::string base_name_color(kColorTopicBaseNames[i]);
      color_camera_info_subs_.emplace_back(
        std::make_shared<::message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
          this, base_name_color + "/camera_info", input_qos_profile));

      color_image_subs_.emplace_back(
        std::make_shared<nvidia::isaac_ros::nitros::message_filters::Subscriber<NitrosView>>(
          this, base_name_color + "/image",
          input_qos_profile));
      if (params_.use_segmentation) {
        const std::string base_name_seg_color(kSegTopicBaseNames[i]);
        segmentation_camera_info_subs_.emplace_back(
          std::make_shared<::message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
            this, base_name_seg_color + "/camera_info", input_qos_profile));

        segmentation_image_subs_.emplace_back(
          std::make_shared<nvidia::isaac_ros::nitros::message_filters::Subscriber<NitrosView>>(
            this, base_name_seg_color + "/image",
            input_qos_profile));
        // Sync color and segmentation images with their camera infos
        timesync_color_mask_.emplace_back(
          std::make_shared<image_mask_exact_sync>(
            image_mask_exact_sync(kQueueSize), *color_image_subs_.back(),
            *color_camera_info_subs_.back(),
            *segmentation_image_subs_.back(), *segmentation_camera_info_subs_.back()));

        timesync_color_mask_.back()->registerCallback(
          std::bind(
            &NvbloxNode::colorPlusMaskImageCallback, this,
            std::placeholders::_1, std::placeholders::_2,
            std::placeholders::_3, std::placeholders::_4));
      } else {
        // Sync only color image with its camera info
        timesync_color_.emplace_back(
          std::make_shared<image_exact_sync>(
            image_exact_sync(kQueueSize), *color_image_subs_.back(),
            *color_camera_info_subs_.back()));

        timesync_color_.back()->registerCallback(
          std::bind(
            &NvbloxNode::colorImageCallback, this,
            std::placeholders::_1, std::placeholders::_2));
      }
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
  // Static esdf
  static_esdf_pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/static_esdf_pointcloud", 1);
  static_map_slice_publisher_ =
    create_publisher<nvblox_msgs::msg::DistanceMapSlice>("~/static_map_slice", 1);

  static_occupancy_grid_publisher_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("~/static_occupancy_grid", 1);
  dynamic_occupancy_grid_publisher_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("~/dynamic_occupancy_grid", 1);
  combined_occupancy_grid_publisher_ =
    create_publisher<nav_msgs::msg::OccupancyGrid>("~/combined_occupancy_grid", 1);

  // Debug outputs
  esdf_slice_bounds_publisher_ =
    create_publisher<visualization_msgs::msg::Marker>("~/esdf_slice_bounds", 1);
  workspace_bounds_publisher_ =
    create_publisher<visualization_msgs::msg::Marker>("~/workspace_bounds", 1);
  shapes_to_clear_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/shapes_to_clear", 1);
  tsdf_zero_crossings_pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/groundplane_estimator_ground_pointcloud", 1);
  tsdf_zero_crossings_ground_plane_publisher_ =
    create_publisher<visualization_msgs::msg::Marker>("~/groundplane_estimator_estimated_plane", 1);

  if (params_.output_pessimistic_distance_map) {
    pessimistic_static_esdf_pointcloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("~/pessimistic_static_esdf_pointcloud", 1);
    pessimistic_static_map_slice_publisher_ =
      create_publisher<nvblox_msgs::msg::DistanceMapSlice>("~/pessimistic_static_map_slice", 1);
  }

  if (isHumanMapping(params_.mapping_type)) {
    // Add additional debug output for human mapping
    color_frame_overlay_publisher_ =
      create_publisher<sensor_msgs::msg::Image>("~/dynamic_color_frame_overlay", 1);
  }
  if (isUsingHumanOrDynamicMapper(params_.mapping_type)) {
    // Debug output for dynamics
    dynamic_points_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("~/dynamic_points", 1);
    dynamic_depth_frame_overlay_publisher_ =
      create_publisher<sensor_msgs::msg::Image>("~/dynamic_depth_frame_overlay", 1);
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
    rmw_qos_profile_services_default);
  save_map_service_ = create_service<nvblox_msgs::srv::FilePath>(
    "~/save_map",
    std::bind(&NvbloxNode::saveMap, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default);
  load_map_service_ = create_service<nvblox_msgs::srv::FilePath>(
    "~/load_map",
    std::bind(&NvbloxNode::loadMap, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default);
  save_rates_service_ = create_service<nvblox_msgs::srv::FilePath>(
    "~/save_rates",
    std::bind(&NvbloxNode::saveRates, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default);
  save_timings_service_ = create_service<nvblox_msgs::srv::FilePath>(
    "~/save_timings",
    std::bind(&NvbloxNode::saveTimings, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default);
  send_esdf_and_gradient_service_ = create_service<nvblox_msgs::srv::EsdfAndGradients>(
    "~/get_esdf_and_gradient",
    std::bind(
      &NvbloxNode::getEsdfAndGradientService, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default);
}

void NvbloxNode::setupTimers()
{
  RCLCPP_INFO_STREAM(get_logger(), "NvbloxNode::setupTimers()");

  queue_processing_timer_ =
    create_wall_timer(
    std::chrono::duration<double>(params_.tick_period_ms / 1000.0),
    std::bind(&NvbloxNode::tick, this), group_processing_);
}

void NvbloxNode::depthPlusMaskImageCallback(
  const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & depth_image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & depth_camera_info,
  const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & seg_image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & seg_camera_info)
{
  timing::Timer tick_timer("ros/depth_image_callback");
  timing::Rates::tick("ros/depth_image_callback");

  const NitrosView & depth_image_view = nvidia::isaac_ros::nitros::NitrosImageView(*depth_image);
  const NitrosView & seg_image_view = nvidia::isaac_ros::nitros::NitrosImageView(*seg_image);

  timing::Delays::tick(
    "ros/depth_image_callback",
    nvblox::Time(getTimestamp(depth_image_view).nanoseconds()),
    nvblox::Time(now().nanoseconds()));

  pushOntoQueue<ImageTypeVariant>(
    "depth_queue",
    std::make_tuple(
      std::make_shared<NitrosView>(depth_image_view), depth_camera_info,
      std::make_shared<NitrosView>(seg_image_view), seg_camera_info),
    depth_image_queue_, &depth_queue_mutex_);
}

void NvbloxNode::depthImageCallback(
  const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & depth_image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & depth_camera_info)
{
  timing::Timer tick_timer("ros/depth_image_callback");
  timing::Rates::tick("ros/depth_image_callback");

  const NitrosView & depth_image_view = nvidia::isaac_ros::nitros::NitrosImageView(*depth_image);

  timing::Delays::tick(
    "ros/depth_image_callback",
    nvblox::Time(
      rclcpp::Time(getTimestamp(depth_image_view)).nanoseconds()),
    nvblox::Time(now().nanoseconds()));

  pushOntoQueue<ImageTypeVariant>(
    "depth_queue",
    std::make_tuple(std::make_shared<NitrosView>(depth_image_view), depth_camera_info),
    depth_image_queue_, &depth_queue_mutex_);
}

void NvbloxNode::colorPlusMaskImageCallback(
  const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & color_image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & color_camera_info,
  const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & seg_image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & seg_camera_info)
{
  timing::Timer tick_timer("ros/color_image_callback");
  timing::Rates::tick("ros/color_image_callback");

  const NitrosView & color_image_view = nvidia::isaac_ros::nitros::NitrosImageView(*color_image);
  const NitrosView & seg_image_view = nvidia::isaac_ros::nitros::NitrosImageView(*seg_image);

  timing::Delays::tick(
    "ros/color_image_callback",
    nvblox::Time(
      rclcpp::Time(getTimestamp(color_image_view)).nanoseconds()),
    nvblox::Time(now().nanoseconds()));

  pushOntoQueue<ImageTypeVariant>(
    "color_mask_queue",
    std::make_tuple(
      std::make_shared<NitrosView>(color_image_view), color_camera_info,
      std::make_shared<NitrosView>(seg_image_view), seg_camera_info),
    color_image_queue_, &color_queue_mutex_);
}

void NvbloxNode::colorImageCallback(
  const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & color_image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & color_camera_info)
{
  timing::Timer tick_timer("ros/color_image_callback");
  timing::Rates::tick("ros/color_image_callback");

  const NitrosView & color_image_view = nvidia::isaac_ros::nitros::NitrosImageView(*color_image);

  timing::Delays::tick(
    "ros/color_image_callback",
    nvblox::Time(
      rclcpp::Time(getTimestamp(color_image_view)).nanoseconds()),
    nvblox::Time(now().nanoseconds()));

  pushOntoQueue<ImageTypeVariant>(
    "color_queue",
    std::make_tuple(std::make_shared<NitrosView>(color_image_view), color_camera_info),
    color_image_queue_, &color_queue_mutex_);
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

  pushOntoQueue(
    kPointcloudQueueName, pointcloud, pointcloud_queue_,
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

  // Process all service calls
  processServiceRequestTaskQueue();

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
    (isUsingHumanOrDynamicMapper(params_.mapping_type)) &&
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
    shouldProcess(now, publish_layer_last_time_, params_.publish_layer_rate_hz))
  {
    publishLayers();
    publish_layer_last_time_ = now;
  }

  if (const rclcpp::Time now = this->get_clock()->now();
    shouldProcess(now, publish_debug_vis_last_time_, params_.publish_debug_vis_rate_hz))
  {
    publishDebugVisualizations();
    publish_debug_vis_last_time_ = now;
  }

  // nvblox statistics
  auto & clk = *get_clock();
  if (params_.print_timings_to_console) {
    RCLCPP_INFO_STREAM_THROTTLE(
      get_logger(), clk, params_.print_statistics_on_console_period_ms,
      "Timing statistics: \n" <<
        nvblox::timing::Timing::Print());
  }
  if (params_.print_rates_to_console) {
    RCLCPP_INFO_STREAM_THROTTLE(
      get_logger(), clk, params_.print_statistics_on_console_period_ms,
      "Rates statistics: \n" <<
        nvblox::timing::Rates::Print());
  }
  if (params_.print_delays_to_console) {
    RCLCPP_INFO_STREAM_THROTTLE(
      get_logger(), clk, params_.print_statistics_on_console_period_ms,
      "Delay statistics: \n" <<
        nvblox::timing::Delays::Print());
  }

  // Restart the idle timer
  idle_timer_.Start();
}

bool NvbloxNode::isPoseAvailable(const ImageTypeVariant & variant_msg)
{
  return std::visit(
    Visitor{
      // Image
      [this](const ImageMsgTuple & msg) -> bool {
        NitrosView & img_msg = *(std::get<kMsgTupleImageIdx>(msg));
        return this->canTransform(img_msg.GetFrameId(), getTimestamp(img_msg));
      },
      // Image + Mask
      [this](const ImageSegmentationMaskMsgTuple & msg) -> bool {
        const NitrosView & img_msg = *std::get<kMsgTupleImageIdx>(msg);
        const NitrosView & mask_msg = *std::get<kMsgTupleMaskIdx>(msg);
        return this->canTransform(img_msg.GetFrameId(), getTimestamp(img_msg)) &&
               this->canTransform(mask_msg.GetFrameId(), getTimestamp(mask_msg));
      }
    }, variant_msg);
}

void NvbloxNode::processDepthQueue()
{
  auto message_ready = [this](const ImageTypeVariant & variant_msg) -> bool {
      return this->isPoseAvailable(variant_msg);
    };

  auto process_image_msg = [this](const ImageTypeVariant & msg) -> bool {
      this->processDepthImage(msg);
      return true;
    };

  processQueue<ImageTypeVariant>(
    depth_image_queue_,    // NOLINT
    &depth_queue_mutex_,    // NOLINT
    message_ready,          // NOLINT
    process_image_msg);
}

void NvbloxNode::processColorQueue()
{
  auto message_ready = [this](const ImageTypeVariant & variant_msg) -> bool {
      return this->isPoseAvailable(variant_msg);
    };

  auto process_image_msg = [this](const ImageTypeVariant & msg) -> bool {
      this->processColorImage(msg);
      return true;
    };

  processQueue<ImageTypeVariant>(
    color_image_queue_,    // NOLINT
    &color_queue_mutex_,    // NOLINT
    message_ready,          // NOLINT
    process_image_msg);
}

void NvbloxNode::processPointcloudQueue()
{
  using PointcloudMsg = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
  auto message_ready = [this](const PointcloudMsg & msg) {
      return this->canTransform(msg->header.frame_id, msg->header.stamp);
    };
  processQueue<PointcloudMsg>(
    pointcloud_queue_,          // NOLINT
    &pointcloud_queue_mutex_,    // NOLINT
    message_ready,             // NOLINT
    std::bind(&NvbloxNode::processLidarPointcloud, this, std::placeholders::_1));
}

void NvbloxNode::processServiceRequestTaskQueue()
{
  // Services are always ready to be processed.
  auto service_ready = [](auto &) -> bool {
      return true;
    };
  // When processing a service request, we execute the corresponding task.
  auto task = [](auto item) -> bool {return item->executeTask();};
  processQueue<EsdfServiceQueuedType>(
    esdf_service_queue_,
    &esdf_service_queue_mutex_,
    service_ready,
    task);
  processQueue<FilePathServiceQueuedType>(
    file_path_service_queue_,
    &file_path_service_queue_mutex_,
    service_ready,
    task);
  // If a service requested visualization, publish right now.
  if (publish_layers_requested_) {
    publishLayers();
    publish_layers_requested_ = false;
    publish_layer_last_time_ = this->get_clock()->now();
  }
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

  if (params_.esdf_mode == EsdfMode::k2D) {
    timing::Timer esdf_output_timer("ros/esdf/slice_output");

    sliceAndPublishEsdf(
      "static", static_mapper_, static_esdf_pointcloud_publisher_,
      static_map_slice_publisher_, static_occupancy_grid_publisher_,
      params_.distance_map_unknown_value_optimistic);

    if (isUsingHumanOrDynamicMapper(params_.mapping_type)) {
      sliceAndPublishEsdf(
        "dynamic", dynamic_mapper_, dynamic_esdf_pointcloud_publisher_,
        dynamic_map_slice_publisher_, dynamic_occupancy_grid_publisher_,
        params_.distance_map_unknown_value_optimistic);
      sliceAndPublishEsdf(
        "combined_dynamic", static_mapper_, combined_esdf_pointcloud_publisher_,
        combined_map_slice_publisher_, combined_occupancy_grid_publisher_,
        params_.distance_map_unknown_value_optimistic, dynamic_mapper_.get());
    }

    if (params_.output_pessimistic_distance_map) {
      sliceAndPublishEsdf(
        "pessimistic_static", static_mapper_,
        pessimistic_static_esdf_pointcloud_publisher_,
        pessimistic_static_map_slice_publisher_, nullptr,
        params_.distance_map_unknown_value_pessimistic);
    }
  }
}

void NvbloxNode::sliceAndPublishEsdf(
  const std::string & name, const std::shared_ptr<Mapper> & mapper,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pointcloud_publisher,
  const rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr & slice_publisher,
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & occupancy_grid_publisher,
  const float unknown_value, const Mapper * mapper_2)
{
  // If anyone wants a slice
  if (pointcloud_publisher->get_subscription_count() > 0 ||
    (params_.publish_esdf_distance_slice && slice_publisher->get_subscription_count() > 0) ||
    (occupancy_grid_publisher && occupancy_grid_publisher->get_subscription_count() > 0))
  {
    // Get the slice as an image
    timing::Timer slicing_timer("ros/" + name + "/esdf/output/compute");
    AxisAlignedBoundingBox aabb;
    Image<float> map_slice_image(MemoryType::kDevice);
    if (mapper_2 != nullptr) {
      esdf_slice_converter_.sliceLayersToCombinedDistanceImage(
        mapper->esdf_layer(), mapper_2->esdf_layer(),
        mapper->esdf_integrator().esdf_slice_height(),
        mapper_2->esdf_integrator().esdf_slice_height(), unknown_value, &map_slice_image, &aabb);
    } else {
      esdf_slice_converter_.sliceLayerToDistanceImage(
        mapper->esdf_layer(),
        mapper->esdf_integrator().esdf_slice_height(), unknown_value,
        &map_slice_image, &aabb);
    }
    slicing_timer.Stop();

    // Publish a pointcloud of the slice image for visualization in RVIZ
    if (pointcloud_publisher->get_subscription_count() > 0) {
      timing::Timer pointcloud_msg_timer("ros/" + name + "/esdf/output/pointcloud");
      sensor_msgs::msg::PointCloud2 pointcloud_msg;
      esdf_slice_converter_.pointcloudMsgFromSliceImage(
        map_slice_image, aabb, mapper->esdf_integrator().esdf_slice_height(),
        mapper->esdf_layer().voxel_size(),
        unknown_value, &pointcloud_msg);
      pointcloud_msg.header.frame_id = params_.global_frame.get();
      pointcloud_msg.header.stamp = update_esdf_last_time_;
      pointcloud_publisher->publish(pointcloud_msg);
    }

    // Publish the distance map slice (costmap for nav2).
    if (params_.publish_esdf_distance_slice && slice_publisher->get_subscription_count() > 0) {
      timing::Timer slice_msg_timer("ros/" + name + "/esdf/output/slice");
      nvblox_msgs::msg::DistanceMapSlice map_slice_msg;
      esdf_slice_converter_.distanceMapSliceMsgFromSliceImage(
        map_slice_image, aabb, mapper->esdf_integrator().esdf_slice_height(),
        mapper->voxel_size_m(), unknown_value,
        &map_slice_msg);
      map_slice_msg.header.frame_id = params_.global_frame.get();
      map_slice_msg.header.stamp = update_esdf_last_time_;
      slice_publisher->publish(map_slice_msg);
    }

    // Publish an occupancy grid of the map slice
    if (occupancy_grid_publisher && occupancy_grid_publisher->get_subscription_count() > 0) {
      timing::Timer pointcloud_msg_timer("ros/" + name + "/esdf/output/occupancy_grid");

      const int width = map_slice_image.cols();
      const int height = map_slice_image.rows();

      if (width > 0 && height > 0) {
        publishOccupancyGridMsg(
          mapper->esdf_layer().voxel_size(), width, height, aabb.min().x(),
          aabb.min().y(), map_slice_image, unknown_value,
          occupancy_grid_publisher);
      }
    }
  }
}

void NvbloxNode::publishOccupancyGridMsg(
  const float voxel_size, const int width, const int height, const double origin_x_position,
  const double origin_y_position, const nvblox::Image<float> & map_slice_image,
  const float unknown_value,
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & occupancy_grid_publisher)
{
  // Set up the message
  nav_msgs::msg::OccupancyGrid occupancy_grid_msg;

  occupancy_grid_msg.header.frame_id = params_.global_frame.get();
  occupancy_grid_msg.info.map_load_time = get_clock()->now();
  occupancy_grid_msg.header.stamp = occupancy_grid_msg.info.map_load_time;

  occupancy_grid_msg.info.resolution = voxel_size;
  occupancy_grid_msg.info.width = width;
  occupancy_grid_msg.info.height = height;
  occupancy_grid_msg.info.origin.position.x = origin_x_position;
  occupancy_grid_msg.info.origin.position.y = origin_y_position;
  occupancy_grid_msg.info.origin.position.z = 0;
  occupancy_grid_msg.info.origin.orientation.x = 0;
  occupancy_grid_msg.info.origin.orientation.y = 0;
  occupancy_grid_msg.info.origin.orientation.z = 0;
  occupancy_grid_msg.info.origin.orientation.w = 1;
  occupancy_grid_msg.data.resize(width * height, kOccupancyGridUnknownValue);

  // Convert the map slice to occupancy grid.
  esdf_slice_converter_.occupancyGridFromSliceImage(
    map_slice_image, occupancy_grid_msg.data.data(),
    unknown_value);

  // Publish the message
  occupancy_grid_publisher->publish(occupancy_grid_msg);
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

NvbloxNode::ImageMsgOptionalMaskMsgTuple NvbloxNode::decomposeImageTypeVariant(
  const ImageTypeVariant & variant_msg)
{
  // (Optional) message parts.
  NitrosViewPtr img_ptr;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg;
  std::optional<NitrosViewPtr> mask_img_opt;
  std::optional<sensor_msgs::msg::CameraInfo::ConstSharedPtr> mask_camera_info_opt;

  std::visit(
    Visitor{
      // Image
      [&](const ImageMsgTuple & msg) {
        img_ptr = std::get<kMsgTupleImageIdx>(msg);
        camera_info_msg = std::get<kMsgTupleCameraInfoIdx>(msg);
        mask_img_opt = std::nullopt;
        mask_camera_info_opt = std::nullopt;
      },
      // Image + Mask
      [&](const ImageSegmentationMaskMsgTuple & msg) {
        img_ptr = std::get<kMsgTupleImageIdx>(msg);
        camera_info_msg = std::get<kMsgTupleCameraInfoIdx>(msg);
        mask_img_opt = std::get<kMsgTupleMaskIdx>(msg);
        mask_camera_info_opt = std::get<kMsgTupleMaskCameraInfoIdx>(msg);
      }
    }, variant_msg);

  return {img_ptr, camera_info_msg, mask_img_opt, mask_camera_info_opt};
}

bool NvbloxNode::processDepthImage(const ImageTypeVariant & depth_msg)
{
  timing::Timer ros_depth_timer("ros/depth");
  timing::Timer transform_timer("ros/depth/transform");

  auto [depth_img_ptr, depth_camera_info_msg, mask_img_opt,
    mask_camera_info_opt] = decomposeImageTypeVariant(depth_msg);

  const std::string depth_frame = (*depth_img_ptr).GetFrameId();
  std::string mask_frame;
  rclcpp::Time mask_image_timestamp;
  if (mask_img_opt) {
    mask_frame = mask_img_opt.value()->GetFrameId();
    mask_image_timestamp = getTimestamp(*mask_img_opt.value());
  }
  // Check if the message is too soon, and if it is discard it
  const rclcpp::Time depth_image_timestamp = getTimestamp(*depth_img_ptr);

  if (integrate_depth_last_times_.find(depth_frame) == integrate_depth_last_times_.end()) {
    integrate_depth_last_times_[depth_frame] = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  const rclcpp::Time last_depth_image_timestamp = integrate_depth_last_times_[depth_frame];
  if (!shouldProcess(
      depth_image_timestamp, last_depth_image_timestamp,
      params_.integrate_depth_rate_hz))
  {
    // To discard we indicate that the image was processed, without actually integrating it.
    return true;
  }
  if (depth_image_timestamp <= shape_clearing_last_time_) {
    // Discard images older then the last shape clearing (would reintegrate cleared shapes).
    return true;
  }
  // Get the timestamp
  constexpr int64_t kNanoSecondsToMilliSeconds = 1e6;
  nvblox::Time update_time_ms(depth_image_timestamp.nanoseconds() / kNanoSecondsToMilliSeconds);

  // Get the TF for this image.
  if (!transformer_.lookupTransformToGlobalFrame(
      depth_frame, depth_image_timestamp,
      &T_L_C_depth_))
  {
    return false;
  }
  Transform T_L_C_mask;
  Transform T_CM_CD;
  if (mask_img_opt) {
    if (!transformer_.lookupTransformToGlobalFrame(mask_frame, mask_image_timestamp, &T_L_C_mask)) {
      return false;
    }
    T_CM_CD = T_L_C_mask.inverse() * T_L_C_depth_;
  }

  transform_timer.Stop();
  timing::Timer conversions_timer("ros/depth/conversions");

  // Convert camera info message to camera object.
  const Camera depth_camera_ = conversions::cameraFromMessage(*depth_camera_info_msg);
  Camera mask_camera;
  if (mask_img_opt) {
    mask_camera = conversions::cameraFromMessage(*mask_camera_info_opt.value());
  }
  // Convert the depth image.
  if (!conversions::depthImageFromNitrosViewAsync(
      *depth_img_ptr, &depth_image_, get_logger(),
      *cuda_stream_))
  {
    RCLCPP_ERROR(get_logger(), "Failed to transform depth image.");
    return false;
  }
  if (mask_img_opt) {
    if (!conversions::monoImageFromNitrosViewAsync(
        *mask_img_opt.value(), &mask_image_, get_logger(),
        *cuda_stream_))
    {
      RCLCPP_ERROR(get_logger(), "Failed to transform mask image.");
      return false;
    }
  }
  conversions_timer.Stop();
  // Do the actual depth integration
  timing::Timer integration_timer("ros/depth/integrate");
  timing::Rates::tick("ros/depth");
  if (mask_img_opt) {
    multi_mapper_->integrateDepth(
      depth_image_, mask_image_, T_L_C_depth_, T_CM_CD, depth_camera_,
      mask_camera);
  } else {
    multi_mapper_->integrateDepth(depth_image_, T_L_C_depth_, depth_camera_, update_time_ms);
  }
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
  if (isHumanMapping(params_.mapping_type)) {
    timing::Timer depth_overlay_timer("ros/depth/output/human_overlay");
    publishHumanDebugOutput(mask_frame, depth_camera_);
    depth_overlay_timer.Stop();
  }
  // Publish back projected depth image for debugging
  timing::Timer back_projected_depth_timer("ros/depth/output/back_projected_depth");
  if (back_projected_depth_publisher_.count(depth_frame) == 0) {
    back_projected_depth_publisher_[depth_frame] =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/back_projected_depth/" + makeSafeTopicName(depth_frame), 1);
    back_projection_idx_[depth_frame] = 0;
  }
  if (back_projected_depth_publisher_[depth_frame]->get_subscription_count() > 0) {
    // Only back project and publish every n-th depth image
    if (back_projection_idx_[depth_frame]++ % params_.back_projection_subsampling == 0) {
      publishBackProjectedDepth(depth_camera_, depth_frame, depth_image_timestamp);
    }
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
      *cuda_stream_);
    dynamic_depth_frame_overlay_publisher_->publish(img_msg);
  }
}

void NvbloxNode::publishHumanDebugOutput(const std::string & camera_frame_id, const Camera & camera)
{
  timing::Timer ros_human_total_timer("ros/humans");
  timing::Timer ros_human_debug_timer("ros/humans/output/debug");

  // Get a human pointcloud
  if (dynamic_points_publisher_->get_subscription_count() >
    0)
  {
    // Grab the human only image.
    const DepthImage & depth_image_only_humans = multi_mapper_->getLastDepthFrameForeground();
    // Back project
    image_back_projector_.backProjectOnGPU(
      depth_image_only_humans, camera, &human_pointcloud_C_device_,
      dynamic_mapper_->occupancy_integrator().max_integration_distance_m());
    transformPointcloudOnGPU(T_L_C_depth_, human_pointcloud_C_device_, &human_pointcloud_L_device_);
  }

  // Publish the human pointcloud
  if (dynamic_points_publisher_->get_subscription_count() > 0) {
    // Back-project human depth image to pointcloud and publish.
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pointcloud_converter_.pointcloudMsgFromPointcloud(human_pointcloud_L_device_, &pointcloud_msg);
    pointcloud_msg.header.frame_id = params_.global_frame;
    pointcloud_msg.header.stamp = get_clock()->now();
    dynamic_points_publisher_->publish(pointcloud_msg);
  }

  // Publish depth and color overlay
  if (dynamic_depth_frame_overlay_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::Image img_msg;
    const ColorImage & depth_overlay = multi_mapper_->getLastDepthFrameMaskOverlay();
    conversions::imageMessageFromColorImage(
      depth_overlay, camera_frame_id, &img_msg,
      *cuda_stream_);
    dynamic_depth_frame_overlay_publisher_->publish(img_msg);
  }
}

void NvbloxNode::publishBackProjectedDepth(
  const Camera & camera, const std::string & frame, const rclcpp::Time & timestamp)
{
  // Get the pointcloud from the depth image
  image_back_projector_.backProjectOnGPU(
    depth_image_, camera, &pointcloud_C_device_,
    params_.max_back_projection_distance);

  // Send the message
  sensor_msgs::msg::PointCloud2 back_projected_depth_msg;
  pointcloud_converter_.pointcloudMsgFromPointcloud(
    pointcloud_C_device_,
    &back_projected_depth_msg);
  back_projected_depth_msg.header.frame_id = frame;
  back_projected_depth_msg.header.stamp = timestamp;
  back_projected_depth_publisher_[frame]->publish(back_projected_depth_msg);
}

bool NvbloxNode::processColorImage(const ImageTypeVariant & color_msg)
{
  timing::Timer ros_color_timer("ros/color");
  timing::Timer transform_timer("ros/color/transform");

  auto [color_img_ptr, color_camera_info_msg, mask_img_opt,
    mask_camera_info_opt] = decomposeImageTypeVariant(color_msg);

  const std::string color_frame = (*color_img_ptr).GetFrameId();
  std::string mask_frame;
  rclcpp::Time mask_image_timestamp;
  if (mask_img_opt) {
    mask_frame = (*mask_img_opt)->GetFrameId();
    mask_image_timestamp = getTimestamp(*mask_img_opt.value());
  }

  // Check if the message is too soon, and if it is discard it
  const rclcpp::Time color_image_timestamp = getTimestamp(*color_img_ptr);
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
  if (color_image_timestamp <= shape_clearing_last_time_) {
    // Discard images older then the last shape clearing (would reintegrate cleared shapes).
    return true;
  }
  // Get the TF for this image.
  Transform T_L_C;
  if (!transformer_.lookupTransformToGlobalFrame(color_frame, color_image_timestamp, &T_L_C)) {
    return false;
  }
  Transform T_L_C_mask;
  if (mask_img_opt) {
    if (!transformer_.lookupTransformToGlobalFrame(mask_frame, mask_image_timestamp, &T_L_C_mask)) {
      return false;
    }
  }
  transform_timer.Stop();

  timing::Timer color_convert_timer("ros/color/conversion");

  // Convert camera info message to camera object.
  const Camera color_camera = conversions::cameraFromMessage(*color_camera_info_msg);

  // Convert the color image.
  if (!conversions::colorImageFromNitrosViewAsync(
      *color_img_ptr, &color_image_, get_logger(),
      *cuda_stream_))
  {
    RCLCPP_ERROR(get_logger(), "Failed to transform color image.");
    return false;
  }
  if (mask_img_opt) {
    if (!conversions::monoImageFromNitrosViewAsync(
        *mask_img_opt.value(), &mask_image_, get_logger(),
        *cuda_stream_))
    {
      RCLCPP_ERROR(get_logger(), "Failed to transform mask image.");
      return false;
    }
  }
  color_convert_timer.Stop();

  // Integrate.
  timing::Timer color_integrate_timer("ros/color/integrate");
  timing::Rates::tick("ros/color");

  if (mask_img_opt) {
    multi_mapper_->integrateColor(
      color_image_, mask_image_, T_L_C, color_camera);
  } else {
    multi_mapper_->integrateColor(color_image_, T_L_C, color_camera);
  }

  timing::Delays::tick(
    "ros/color_image_integration",
    nvblox::Time(color_image_timestamp.nanoseconds()),
    nvblox::Time(now().nanoseconds()));
  integrate_color_last_times_[color_frame] = color_image_timestamp;
  color_integrate_timer.Stop();
  // TODO(dtingdahl): Renable the code below.
  // if (isHumanMapping(params_.mapping_type)) {
  //   if (color_frame_overlay_publisher_->get_subscription_count() > 0) {
  //     timing::Timer color_overlay_timer("ros/color/output/human_overlay");
  //     sensor_msgs::msg::Image img_msg;
  //     const ColorImage & color_overlay = multi_mapper_->getLastColorFrameMaskOverlay();
  //     conversions::imageMessageFromColorImage(
  //       color_overlay, color_frame, &img_msg,
  //       *cuda_stream_);
  //     color_frame_overlay_publisher_->publish(img_msg);
  //     color_overlay_timer.Stop();
  //   }
  // }
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
  if (pointcloud_timestamp <= shape_clearing_last_time_) {
    // Discard images older then the last shape clearing (would reintegrate cleared shapes).
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
    params_.lidar_width, params_.lidar_height, params_.lidar_min_valid_range_m,
    params_.lidar_max_valid_range_m, params_.min_angle_below_zero_elevation_rad,
    params_.max_angle_above_zero_elevation_rad) :
    Lidar(
    params_.lidar_width, params_.lidar_height, params_.lidar_min_valid_range_m,
    params_.lidar_max_valid_range_m, params_.lidar_vertical_fov_rad);

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
  timing::Timer publish_layer_timer("ros/publish_layers");

  // Find the transform used for exclusing blocks far from the robot
  Transform T_L_C;
  if (!transformer_.lookupTransformToGlobalFrame(
      params_.map_clearing_frame_id,
      rclcpp::Time(0), &T_L_C))
  {
    RCLCPP_INFO_STREAM_THROTTLE(
      get_logger(), *get_clock(),
      kTimeBetweenDebugMessagesMs,
      "Lookup transform failed for frame "
        << params_.map_clearing_frame_id.get()
        << ". Layer pointclouds not published");
    return;
  }

  const std::string frame_id = params_.global_frame.get();
  const rclcpp::Time timestamp = get_clock()->now();
  layer_publisher_->serializeAndpublishSubscribedLayers(
    T_L_C, frame_id, timestamp, params_.layer_streamer_bandwidth_limit_mbps, static_mapper_,
    dynamic_mapper_,
    get_logger());
}

void NvbloxNode::publishDebugVisualizations()
{
  const rclcpp::Time timestamp = get_clock()->now();

  // Ground Plane Estimation: Publish the tsdf zero crossings
  if (tsdf_zero_crossings_pointcloud_publisher_->get_subscription_count() > 0) {
    std::optional<std::vector<Vector3f>> maybe_tsdf_zero_crossings =
      multi_mapper_->ground_plane_estimator().tsdf_zero_crossings_ground_candidates();
    if (maybe_tsdf_zero_crossings) {
      // Convert to Pointcloud -> PointCloud2 -> Publish
      tsdf_zero_crossings_device_.copyFromAsync(
        maybe_tsdf_zero_crossings.value(), CudaStreamOwning());
      sensor_msgs::msg::PointCloud2 tsdf_zero_crossings_pointcloud2_msg;
      pointcloud_converter_.pointcloudMsgFromPointcloud(
        tsdf_zero_crossings_device_, &tsdf_zero_crossings_pointcloud2_msg);

      tsdf_zero_crossings_pointcloud2_msg.header.frame_id = params_.global_frame.get();
      tsdf_zero_crossings_pointcloud2_msg.header.stamp = update_esdf_last_time_;
      tsdf_zero_crossings_pointcloud_publisher_->publish(
        tsdf_zero_crossings_pointcloud2_msg);
    }
  }

  // Ground Plane Estimation: Publish the ground plane
  if (tsdf_zero_crossings_ground_plane_publisher_->get_subscription_count() > 0) {
    std::optional<Plane> maybe_plane = multi_mapper_->ground_plane_estimator().ground_plane();
    if (maybe_plane) {
      Transform T_S_PB;
      if (transformer_.lookupTransformToGlobalFrame(
          params_.ground_plane_visualization_attachment_frame_id,
          rclcpp::Time(0), &T_S_PB))
      {
        const visualization_msgs::msg::Marker marker_plane =
          planeToMarker(
          T_S_PB, maybe_plane.value(),
          params_.ground_plane_visualization_side_length,
          update_esdf_last_time_, params_.global_frame.get());
        tsdf_zero_crossings_ground_plane_publisher_->publish(marker_plane);
      } else {
        RCLCPP_INFO_STREAM_THROTTLE(
          get_logger(), *get_clock(), kTimeBetweenDebugMessagesMs,
          "Tried to publish ground plane but couldn't look up frame: "
            << params_.ground_plane_visualization_attachment_frame_id
            .get());
      }
    }
  }

  // Also publish the esdf slice bounds (showing esdf max/min 2d height)
  if (esdf_slice_bounds_publisher_->get_subscription_count() > 0) {
    // The frame to which the slice limits visualization is attached.
    // We get the transform from the plane-body (PB) frame, to the scene (S).
    Transform T_S_PB;
    if (transformer_.lookupTransformToGlobalFrame(
        params_.esdf_slice_bounds_visualization_attachment_frame_id, rclcpp::Time(0),
        &T_S_PB))
    {
      // Get and publish the planes representing the slice bounds in z.
      const visualization_msgs::msg::Marker marker_bottom = heightLimitToMarker(
        T_S_PB, params_.esdf_slice_bounds_visualization_side_length, timestamp,
        params_.global_frame.get(), static_mapper_->esdf_integrator().esdf_slice_min_height(),
        HeightLimitMarkerType::kBottomHeightLimit);
      const visualization_msgs::msg::Marker marker_top = heightLimitToMarker(
        T_S_PB, params_.esdf_slice_bounds_visualization_side_length, timestamp,
        params_.global_frame.get(), static_mapper_->esdf_integrator().esdf_slice_max_height(),
        HeightLimitMarkerType::kTopHeightLimit);
      esdf_slice_bounds_publisher_->publish(marker_bottom);
      esdf_slice_bounds_publisher_->publish(marker_top);
    } else {
      RCLCPP_INFO_STREAM_THROTTLE(
        get_logger(), *get_clock(), kTimeBetweenDebugMessagesMs,
        "Tried to publish esdf slice bounds but couldn't look up frame: "
          << params_.esdf_slice_bounds_visualization_attachment_frame_id.get());
    }
  }

  // Workspace Bounds
  if (workspace_bounds_publisher_ && workspace_bounds_publisher_->get_subscription_count() > 0) {
    // Get the workspace bounds type and corners.
    const WorkspaceBoundsType bounds_type =
      static_mapper_->tsdf_integrator().view_calculator().workspace_bounds_type();
    const Vector3f min_corner =
      static_mapper_->tsdf_integrator().view_calculator().workspace_bounds_min_corner_m();
    const Vector3f max_corner =
      static_mapper_->tsdf_integrator().view_calculator().workspace_bounds_max_corner_m();
    // Publish visualization of the workspace bounds depending on its type.
    if (bounds_type == WorkspaceBoundsType::kBoundingBox) {
      const visualization_msgs::msg::Marker workspace_bounds_marker =
        boundingBoxToMarker(min_corner, max_corner, timestamp, params_.global_frame.get());
      workspace_bounds_publisher_->publish(workspace_bounds_marker);
    } else if (bounds_type == WorkspaceBoundsType::kHeightBounds) {
      // The frame to which the workspace height bounds visualization is attached.
      // We get the transform from the plane-body (PB) frame, to the scene (S).
      Transform T_S_PB;
      if (transformer_.lookupTransformToGlobalFrame(
          params_.workspace_height_bounds_visualization_attachment_frame_id, rclcpp::Time(0),
          &T_S_PB))
      {
        // Get and publish the planes representing the workspace height bounds in z.
        const visualization_msgs::msg::Marker marker_bottom = heightLimitToMarker(
          T_S_PB, params_.workspace_height_bounds_visualization_side_length, timestamp,
          params_.global_frame.get(), min_corner.z(), HeightLimitMarkerType::kBottomHeightLimit);
        const visualization_msgs::msg::Marker marker_top = heightLimitToMarker(
          T_S_PB, params_.workspace_height_bounds_visualization_side_length, timestamp,
          params_.global_frame.get(), max_corner.z(), HeightLimitMarkerType::kTopHeightLimit);
        workspace_bounds_publisher_->publish(marker_bottom);
        workspace_bounds_publisher_->publish(marker_top);
      } else {
        RCLCPP_INFO_STREAM_THROTTLE(
          get_logger(), *get_clock(), kTimeBetweenDebugMessagesMs,
          "Tried to publish workspace height bounds but couldn't look up frame: "
            << params_.workspace_height_bounds_visualization_attachment_frame_id.get());
      }
    }
  }
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
  // Define the task function
  TaskFunctionType<NvbloxNode, nvblox_msgs::srv::FilePath> request_task =
    [](auto node,
    auto service_request,
    auto service_response) {
      // If we get a full path, then write to that path.
      bool success = true;
      if (ends_with(service_request->file_path, ".ply")) {
        // Make sure the mesh is computed
        node->static_mapper_->updateColorMesh(UpdateFullLayer::kYes);
        success = io::outputColorMeshLayerToPly(
          node->static_mapper_->color_mesh_layer(), service_request->file_path);
      } else {
        // If we get a partial path then output a bunch of stuff to a folder.
        success &= io::outputVoxelLayerToPly(
          node->static_mapper_->tsdf_layer(),
          service_request->file_path + "/ros2_tsdf.ply");
        success &= io::outputVoxelLayerToPly(
          node->static_mapper_->esdf_layer(),
          service_request->file_path + "/ros2_esdf.ply");
        success &= io::outputColorMeshLayerToPly(
          node->static_mapper_->color_mesh_layer(),
          service_request->file_path + "/ros2_mesh.ply");
        if (isDynamicMapping(node->params_.mapping_type)) {
          success &= io::outputVoxelLayerToPly(
            node->static_mapper_->freespace_layer(),
            service_request->file_path + "/ros2_freespace.ply");
        }
      }
      if (success) {
        RCLCPP_INFO_STREAM(
          node->get_logger(), "Output PLY file(s) to " << service_request->file_path);
      } else {
        RCLCPP_WARN_STREAM(
          node->get_logger(), "Failed to write PLY file(s) to " << service_request->file_path);
      }
      service_response->success = success;
      return success;
    };

  // Create the ServiceRequestTask
  auto task =
    std::make_shared<ServiceRequestTask<NvbloxNode, nvblox_msgs::srv::FilePath>>(
    request_task, this, request, response);

  // Push the task onto the queue and wait for completion.
  pushOntoQueue(
    kFilePathServiceQueueName, task, file_path_service_queue_,
    &esdf_service_queue_mutex_);
  task->waitForTaskCompletion();
}

void NvbloxNode::saveMap(
  const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
  std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response)
{
  // Define the task function
  TaskFunctionType<NvbloxNode, nvblox_msgs::srv::FilePath> request_task =
    [](auto node,
    auto service_request,
    auto service_response) {
      std::string filename = service_request->file_path;
      if (!ends_with(service_request->file_path, ".nvblx")) {
        filename += ".nvblx";
      }

      service_response->success = node->static_mapper_->saveLayerCake(filename);
      if (service_response->success) {
        RCLCPP_INFO_STREAM(node->get_logger(), "Output map to file to " << filename);
      } else {
        RCLCPP_WARN_STREAM(node->get_logger(), "Failed to write file to " << filename);
      }
      return service_response->success;
    };

  // Create the ServiceRequestTask
  auto task =
    std::make_shared<ServiceRequestTask<NvbloxNode, nvblox_msgs::srv::FilePath>>(
    request_task, this, request, response);

  // Push the task onto the queue and wait for completion.
  pushOntoQueue(
    kFilePathServiceQueueName, task, file_path_service_queue_,
    &esdf_service_queue_mutex_);
  task->waitForTaskCompletion();
}

void NvbloxNode::loadMap(
  const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
  std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response)
{
  // Define the task function
  TaskFunctionType<NvbloxNode, nvblox_msgs::srv::FilePath> request_task =
    [](auto node,
    auto service_request,
    auto service_response) {
      std::string filename = service_request->file_path;
      if (!ends_with(service_request->file_path, ".nvblx")) {
        filename += ".nvblx";
      }

      service_response->success = node->static_mapper_->loadMap(filename);
      if (service_response->success) {
        RCLCPP_INFO_STREAM(node->get_logger(), "Loaded map to file from " << filename);
      } else {
        RCLCPP_WARN_STREAM(node->get_logger(), "Failed to load map file from " << filename);
      }
      return service_response->success;
    };

  // Create the ServiceRequestTask
  auto task =
    std::make_shared<ServiceRequestTask<NvbloxNode, nvblox_msgs::srv::FilePath>>(
    request_task, this, request, response);

  // Push the task onto the queue and wait for completion.
  pushOntoQueue(
    kFilePathServiceQueueName, task, file_path_service_queue_,
    &esdf_service_queue_mutex_);
  task->waitForTaskCompletion();
}

void NvbloxNode::saveRates(
  const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
  std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response)
{
  // NOTE: We do not add this service call to a service call queue
  // as all functions called in here are very short and thread safe.

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
  // NOTE: We do not add this service call to a service call queue
  // as all functions called in here are very short and thread safe.

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
  timing::Timer esdf_service_timer("ros/esdf_service");
  timing::Rates::tick("ros/esdf_service");
  RCLCPP_INFO(
    get_logger(),
    "\nReceived request for ESDF with:\nupdate_esdf: %d\nuse_aabb: "
    "%d\nvisualize_esdf: %d\naabb_min_m: [%0.2f, %0.2f, %0.2f],"
    "\naabb_size_m: [%0.2f, %0.2f, %0.2f]",
    request->update_esdf, request->use_aabb, request->visualize_esdf, request->aabb_min_m.x,
    request->aabb_min_m.y, request->aabb_min_m.z,
    request->aabb_size_m.x, request->aabb_size_m.y,
    request->aabb_size_m.z);
  if (params_.esdf_mode != EsdfMode::k3D) {
    RCLCPP_FATAL_STREAM(
      get_logger(),
      "The ESDF service is only intended for mapping with 3D "
      "ESDFs. You're in 2D mode."
      "To use this function set esdf_mode: 3d. Exiting.");
    exit(1);
  }

  // Define the task function
  TaskFunctionType<NvbloxNode, nvblox_msgs::srv::EsdfAndGradients> request_task =
    [](auto node,
    auto service_request,
    auto service_response) {
      // NOTE: We could consider to enable requests in other
      // frame ids by re-sampling/interpolating the grid in the future.
      timing::Timer create_esdf_grid_timer("ros/esdf_service/task_function");
      if (service_request->frame_id != node->params_.global_frame.get()) {
        RCLCPP_WARN_STREAM(
          node->get_logger(),
          "Requested EsdfAndGradients in " << service_request->frame_id
                                           << " frame but nvblox is mapping in " <<
            node->params_.global_frame.get() << " frame. Sending empty grid.");
        service_response->success = false;
        return false;
      }

      // Clear shapes in the tsdf and visualize them in rviz if requested.
      timing::Timer clear_shapes_timer("ros/esdf_service/task_function/clear_shapes");
      std::vector<BoundingShape> shapes_to_clear =
        conversions::getShapesToClear(service_request, node->get_logger());
      if (node->shapes_to_clear_publisher_ &&
        node->shapes_to_clear_publisher_->get_subscription_count() > 0)
      {
        timing::Timer clear_shapes_vis_timer("ros/esdf_service/task_function/clear_shapes/vis");
        const visualization_msgs::msg::MarkerArray marker_array =
          boundingShapesToMarker(
          shapes_to_clear, node->get_clock()->now(),
          node->params_.global_frame.get(), node->get_logger());
        node->shapes_to_clear_publisher_->publish(marker_array);
        clear_shapes_vis_timer.Stop();
      }
      if (shapes_to_clear.size() > 0) {
        node->static_mapper_->clearTsdfInsideShapes(shapes_to_clear);
        node->shape_clearing_last_time_ = node->get_clock()->now();
      }
      clear_shapes_timer.Stop();

      // Update the Esdf layer.
      if (service_request->update_esdf) {
        timing::Timer update_esdf_timer("ros/esdf_service/task_function/update_esdf");
        node->processEsdf();
        node->update_esdf_last_time_ = node->get_clock()->now();
        update_esdf_timer.Stop();
      }
      node->publish_layers_requested_ = service_request->visualize_esdf;
      // Construct the bounding box.
      timing::Timer create_response_timer("ros/esdf_service/task_function/create_response");
      node->esdf_and_gradients_converter_.getEsdfAndGradientResponse(
        node->static_mapper_->esdf_layer(),
        node->params_.esdf_and_gradients_unobserved_value,
        service_request, service_response, *node->cuda_stream_);
      service_response->header.stamp = node->newest_integrated_depth_time_;
      service_response->header.frame_id = node->params_.global_frame.get();
      if (service_response->success) {
        RCLCPP_INFO_STREAM(
          node->get_logger(),
          "Successfully wrote requested ESDF to MultiArrayMsg.");
      } else {
        RCLCPP_WARN_STREAM(
          node->get_logger(),
          "Writing requested ESDF to MultiArrayMsg failed. Is your AABB definition valid?");
      }
      create_response_timer.Stop();
      return true;
    };

  // Create the ServiceRequestTask
  auto task =
    std::make_shared<ServiceRequestTask<NvbloxNode, nvblox_msgs::srv::EsdfAndGradients>>(
    request_task, this, request, response);

  // Push the task onto the queue and wait for completion.
  pushOntoQueue(kEsdfServiceQueueName, task, esdf_service_queue_, &esdf_service_queue_mutex_);
  task->waitForTaskCompletion();
}

}  // namespace nvblox

// Register the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvblox::NvbloxNode)
