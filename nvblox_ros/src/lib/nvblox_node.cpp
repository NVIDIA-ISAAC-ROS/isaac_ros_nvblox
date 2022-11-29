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

#include <nvblox/core/cuda/warmup.h>
#include <nvblox/io/mesh_io.h>
#include <nvblox/io/pointcloud_io.h>
#include <nvblox/utils/timing.h>

#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "nvblox_ros/conversions.hpp"
#include "nvblox_ros/qos.hpp"
#include "nvblox_ros/visualization.hpp"

namespace nvblox
{

NvbloxNode::NvbloxNode()
: Node("nvblox_node"), transformer_(this)
{
  // Declare & initialize the parameters.
  voxel_size_ = declare_parameter<float>("voxel_size", voxel_size_);
  global_frame_ = declare_parameter<std::string>("global_frame", global_frame_);
  pose_frame_ = declare_parameter<std::string>("pose_frame", pose_frame_);
  mesh_ = declare_parameter<bool>("mesh", mesh_);
  esdf_ = declare_parameter<bool>("esdf", esdf_);
  esdf_2d_ = declare_parameter<bool>("esdf_2d", esdf_2d_);
  distance_slice_ = declare_parameter<bool>("distance_slice", distance_slice_);
  use_color_ = declare_parameter<bool>("use_color", use_color_);
  use_depth_ = declare_parameter<bool>("use_depth", use_depth_);
  use_lidar_ = declare_parameter<bool>("use_lidar", use_lidar_);
  slice_height_ = declare_parameter<float>("slice_height", slice_height_);
  min_height_ = declare_parameter<float>("min_height", min_height_);
  max_height_ = declare_parameter<float>("max_height", max_height_);
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
  max_tsdf_update_hz_ =
    declare_parameter<float>("max_tsdf_update_hz", max_tsdf_update_hz_);
  max_color_update_hz_ =
    declare_parameter<float>("max_color_update_hz", max_color_update_hz_);
  max_pointcloud_update_hz_ = declare_parameter<float>(
    "max_pointcloud_update_hz", max_pointcloud_update_hz_);
  max_mesh_update_hz_ =
    declare_parameter<float>("max_mesh_update_hz", max_mesh_update_hz_);
  max_esdf_update_hz_ =
    declare_parameter<float>("max_esdf_update_hz", max_esdf_update_hz_);
  max_poll_rate_hz_ =
    declare_parameter<float>("max_poll_rate_hz", max_poll_rate_hz_);

  // Settings for QoS.
  const std::string kDefaultQoS = "SYSTEM_DEFAULT";
  std::string depth_qos =
    declare_parameter<std::string>("depth_qos", kDefaultQoS);
  std::string color_qos =
    declare_parameter<std::string>("color_qos", kDefaultQoS);

  // Settings for map clearing
  map_clearing_radius_m_ =
    declare_parameter<float>("map_clearing_radius_m", map_clearing_radius_m_);

  // Set the transformer settings.
  transformer_.set_global_frame(global_frame_);
  transformer_.set_pose_frame(pose_frame_);

  // Initialize the map
  mapper_ = std::make_unique<RgbdMapper>(voxel_size_);

  // Create callback groups, which allows processing to go in parallel with the
  // subscriptions.
  group_processing_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  constexpr int kQueueSize = 10;

  if (!use_depth_ && !use_lidar_) {
    RCLCPP_WARN(
      get_logger(),
      "Nvblox is running without depth or lidar input, the cost maps and"
      " reconstructions will not update");
  }

  if (use_depth_) {
    // Subscribe to synchronized depth + cam_info topics
    depth_sub_.subscribe(this, "depth/image", parseQoSString(depth_qos));
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
    color_sub_.subscribe(this, "color/image", parseQoSString(color_qos));
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
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", kQueueSize,
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

  // Create a timer for processing incoming messages.
  // In case the rates are 0.0 (uncapped), set them to the max poll rate.
  double effective_esdf_rate_hz = max_esdf_update_hz_;
  if (effective_esdf_rate_hz <= 0.0) {
    effective_esdf_rate_hz = max_poll_rate_hz_;
  }
  double effective_mesh_rate_hz = max_mesh_update_hz_;
  if (effective_mesh_rate_hz <= 0.0) {
    effective_mesh_rate_hz = max_poll_rate_hz_;
  }
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
    std::chrono::duration<double>(1.0 / effective_esdf_rate_hz),
    std::bind(&NvbloxNode::processEsdf, this), group_processing_);
  mesh_processing_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / effective_mesh_rate_hz),
    std::bind(&NvbloxNode::processMesh, this), group_processing_);

  // Publishers
  mesh_publisher_ = create_publisher<nvblox_msgs::msg::Mesh>("~/mesh", 1);
  pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/pointcloud", 1);
  map_slice_publisher_ =
    create_publisher<nvblox_msgs::msg::DistanceMapSlice>("~/map_slice", 1);
  mesh_marker_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/mesh_marker",
    1);
  slice_bounds_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
    "~/map_slice_bounds", 1);

  // Services
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

  // Integrator settings.
  mapper_->tsdf_integrator().max_integration_distance_m(
    declare_parameter<float>(
      "tsdf_integrator_max_integration_distance_m",
      mapper_->tsdf_integrator().max_integration_distance_m()));
  mapper_->lidar_tsdf_integrator().max_integration_distance_m(
    declare_parameter<float>(
      "lidar_tsdf_integrator_max_integration_distance_m",
      mapper_->lidar_tsdf_integrator().max_integration_distance_m()));
  // These parameters are shared between the LIDAR and depth integrator or
  // things get too complicated.
  mapper_->tsdf_integrator().truncation_distance_vox(
    declare_parameter<float>(
      "tsdf_integrator_truncation_distance_vox",
      mapper_->tsdf_integrator().truncation_distance_vox()));
  // Copy over to the LIDAR.
  mapper_->lidar_tsdf_integrator().truncation_distance_vox(
    mapper_->tsdf_integrator().truncation_distance_vox());
  mapper_->tsdf_integrator().max_weight(
    declare_parameter<float>(
      "tsdf_integrator_max_weight", mapper_->tsdf_integrator().max_weight()));
  // Copy over to the LIDAR.
  mapper_->lidar_tsdf_integrator().max_weight(
    mapper_->tsdf_integrator().max_weight());
  mapper_->mesh_integrator().min_weight(
    declare_parameter<float>(
      "mesh_integrator_min_weight", mapper_->mesh_integrator().min_weight()));
  mapper_->mesh_integrator().weld_vertices(
    declare_parameter<bool>(
      "mesh_integrator_weld_vertices",
      mapper_->mesh_integrator().weld_vertices()));
  mapper_->color_integrator().max_integration_distance_m(
    declare_parameter<float>(
      "color_integrator_max_integration_distance_m",
      mapper_->color_integrator().max_integration_distance_m()));
  mapper_->esdf_integrator().min_weight(
    declare_parameter<float>(
      "esdf_integrator_min_weight", mapper_->esdf_integrator().min_weight()));
  mapper_->esdf_integrator().max_site_distance_vox(
    declare_parameter<float>(
      "esdf_integrator_max_site_distance_vox",
      mapper_->esdf_integrator().max_site_distance_vox()));
  mapper_->esdf_integrator().max_distance_m(
    declare_parameter<float>(
      "esdf_integrator_max_distance_m",
      mapper_->esdf_integrator().max_distance_m()));

  // Where to put saved stuff
  output_dir_ = declare_parameter<std::string>("output_dir", output_dir_);

  RCLCPP_INFO_STREAM(
    get_logger(),
    "Outputting results (as requested) to: " << output_dir_);

  // Start the message statistics
  depth_frame_statistics_.Start();
  rgb_frame_statistics_.Start();

  RCLCPP_INFO_STREAM(
    get_logger(), "Started up nvblox node in frame " <<
      global_frame_ << " and voxel size " <<
      voxel_size_);

  // Set state.
  last_tsdf_update_time_ = rclcpp::Time(0ul, get_clock()->get_clock_type());
  last_color_update_time_ = rclcpp::Time(0ul, get_clock()->get_clock_type());
  last_pointcloud_update_time_ =
    rclcpp::Time(0ul, get_clock()->get_clock_type());
  last_esdf_update_time_ = rclcpp::Time(0ul, get_clock()->get_clock_type());
  last_mesh_update_time_ = rclcpp::Time(0ul, get_clock()->get_clock_type());
}

void NvbloxNode::depthImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_img_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  // Message statistics
  depth_frame_statistics_.OnMessageReceived(
    *depth_img_ptr,
    get_clock()->now().nanoseconds());
  constexpr int kPublishPeriodMs = 10000;
  auto & clk = *get_clock();
  RCLCPP_INFO_STREAM_THROTTLE(
    get_logger(), clk, kPublishPeriodMs,
    "Depth frame statistics: \n" <<
      libstatistics_collector::moving_average_statistics::
      StatisticsDataToString(
      depth_frame_statistics_.GetStatisticsResults()));

  RCLCPP_INFO_STREAM_THROTTLE(
    get_logger(), clk, kPublishPeriodMs,
    "Timing statistics: \n" <<
      nvblox::timing::Timing::Print());

  timing::Timer ros_total_timer("ros/total");

  // Push it into the queue.
  {
    const std::lock_guard<std::mutex> lock(depth_queue_mutex_);
    depth_image_queue_.emplace_back(depth_img_ptr, camera_info_msg);
  }
}

void NvbloxNode::colorImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & color_image_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  // Message statistics
  rgb_frame_statistics_.OnMessageReceived(
    *color_image_ptr,
    get_clock()->now().nanoseconds());
  constexpr int kPublishPeriodMs = 10000;
  auto & clk = *get_clock();
  RCLCPP_INFO_STREAM_THROTTLE(
    get_logger(), clk, kPublishPeriodMs,
    "RGB frame statistics: \n" <<
      libstatistics_collector::moving_average_statistics::
      StatisticsDataToString(
      rgb_frame_statistics_.GetStatisticsResults()));

  timing::Timer ros_total_timer("ros/total");

  // Push it into the queue.
  {
    const std::lock_guard<std::mutex> lock(color_queue_mutex_);
    color_image_queue_.emplace_back(color_image_ptr, camera_info_msg);
  }
}

void NvbloxNode::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud)
{
  constexpr int kPublishPeriodMs = 10000;
  auto & clk = *get_clock();

  RCLCPP_INFO_STREAM_THROTTLE(
    get_logger(), clk, kPublishPeriodMs,
    "Timing statistics: \n" <<
      nvblox::timing::Timing::Print());

  timing::Timer ros_total_timer("ros/total");

  // Push it into the queue. It's converted later on.
  {
    const std::lock_guard<std::mutex> lock(pointcloud_queue_mutex_);
    pointcloud_queue_.emplace_back(pointcloud);
  }
}

void NvbloxNode::processDepthQueue()
{
  timing::Timer ros_total_timer("ros/total");

  // Copy over all the pointers we actually want to process here.
  std::vector<std::pair<sensor_msgs::msg::Image::ConstSharedPtr,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr>>
  images_to_process;

  std::unique_lock<std::mutex> lock(depth_queue_mutex_);

  if (depth_image_queue_.empty()) {
    lock.unlock();
    return;
  }

  auto it_first_valid = depth_image_queue_.end();
  auto it_last_valid = depth_image_queue_.begin();

  for (auto it = depth_image_queue_.begin(); it != depth_image_queue_.end();
    it++)
  {
    sensor_msgs::msg::Image::ConstSharedPtr depth_img_ptr = it->first;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg = it->second;

    rclcpp::Time timestamp = depth_img_ptr->header.stamp;

    // Process this image in the queue
    if (canTransform(depth_img_ptr->header)) {
      images_to_process.push_back(
        std::make_pair(depth_img_ptr, camera_info_msg));
    } else {
      continue;
    }

    // If we processed this frame, keep track of that fact so we can delete it
    // at the end.
    if (it_first_valid == depth_image_queue_.end()) {
      it_first_valid = it;
    }
    if (it_last_valid <= it) {
      it_last_valid = it;
    }
  }

  // Now we have 2 iterators pointing to what we want to delete.
  if (it_first_valid != depth_image_queue_.end()) {
    // Actually erase from the beginning of the queue.
    depth_image_queue_.erase(depth_image_queue_.begin(), ++it_last_valid);
  }
  lock.unlock();

  // Now we actually process the depth images.
  if (images_to_process.empty()) {
    return;
  }

  rclcpp::Time last_timestamp;
  for (auto image_pair : images_to_process) {
    // Cache clock_now.
    last_timestamp = image_pair.first->header.stamp;
    rclcpp::Time clock_now = last_timestamp;

    if (max_tsdf_update_hz_ > 0.0f &&
      (clock_now - last_tsdf_update_time_).seconds() <
      1.0f / max_tsdf_update_hz_)
    {
      // Skip integrating this.
      continue;
    }
    last_tsdf_update_time_ = clock_now;

    processDepthImage(image_pair.first, image_pair.second);
  }

  // Clear map outside radius, if requested
  if (map_clearing_radius_m_ > 0.0f) {
    const auto depth_img_ptr = images_to_process.back().first;
    clearMapOutsideOfRadius(depth_img_ptr->header.frame_id, depth_img_ptr->header.stamp);
  }
}

void NvbloxNode::processPointcloudQueue()
{
  timing::Timer ros_total_timer("ros/total");

  std::unique_lock<std::mutex> lock(pointcloud_queue_mutex_);

  if (pointcloud_queue_.empty()) {
    lock.unlock();
    return;
  }

  std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr>
  pointclouds_to_process;

  auto it_first_valid = pointcloud_queue_.end();
  auto it_last_valid = pointcloud_queue_.begin();

  for (auto it = pointcloud_queue_.begin(); it != pointcloud_queue_.end();
    it++)
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_ptr = *it;

    rclcpp::Time timestamp = pointcloud_ptr->header.stamp;

    // Process this pointcloud in the queue
    if (canTransform(pointcloud_ptr->header)) {
      pointclouds_to_process.push_back(pointcloud_ptr);
    } else {
      continue;
    }

    // If we processed this frame, keep track of that fact so we can delete it
    // at the end.
    if (it_first_valid == pointcloud_queue_.end()) {
      it_first_valid = it;
    }
    if (it_last_valid <= it) {
      it_last_valid = it;
    }
  }

  // Now we have 2 iterators pointing to what we want to delete.
  if (it_first_valid != pointcloud_queue_.end()) {
    // Actually erase from the beginning of the queue.
    pointcloud_queue_.erase(pointcloud_queue_.begin(), ++it_last_valid);
  }
  lock.unlock();

  // Now we actually process the depth images.
  if (pointclouds_to_process.empty()) {
    return;
  }

  rclcpp::Time last_timestamp;
  for (auto pointcloud : pointclouds_to_process) {
    // Cache clock_now.
    last_timestamp = pointcloud->header.stamp;
    rclcpp::Time clock_now = last_timestamp;

    if (max_pointcloud_update_hz_ > 0.0f &&
      (clock_now - last_pointcloud_update_time_).seconds() <
      1.0f / max_pointcloud_update_hz_)
    {
      // Skip integrating this.
      continue;
    }
    last_pointcloud_update_time_ = clock_now;

    processLidarPointcloud(pointcloud);
  }

  // Clear map outside radius, if requested
  if (map_clearing_radius_m_ > 0.0f) {
    const auto pointcloud_ptr = pointclouds_to_process.back();
    clearMapOutsideOfRadius(pointcloud_ptr->header.frame_id, pointcloud_ptr->header.stamp);
  }
}

void NvbloxNode::processEsdf()
{
  timing::Timer ros_total_timer("ros/total");

  // Esdf integrator (if enabled)
  if (esdf_) {
    const rclcpp::Time clock_now = get_clock()->now();
    last_esdf_update_time_ = clock_now;

    // Then do the update.
    // Otherwise do nothing.
    updateEsdf(clock_now);
  }
}

void NvbloxNode::processMesh()
{
  timing::Timer ros_total_timer("ros/total");

  // Mesh integrator
  if (mesh_) {
    const rclcpp::Time clock_now = get_clock()->now();
    last_mesh_update_time_ = clock_now;
    updateMesh(clock_now);
  }
}

void NvbloxNode::processColorQueue()
{
  timing::Timer ros_total_timer("ros/total");

  // Copy over all the pointers we actually want to process here.
  std::vector<std::pair<sensor_msgs::msg::Image::ConstSharedPtr,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr>>
  images_to_process;

  std::unique_lock<std::mutex> lock(color_queue_mutex_);

  if (color_image_queue_.empty()) {
    lock.unlock();
    return;
  }

  auto it_first_valid = color_image_queue_.end();
  auto it_last_valid = color_image_queue_.begin();
  for (auto it = color_image_queue_.begin(); it != color_image_queue_.end();
    it++)
  {
    sensor_msgs::msg::Image::ConstSharedPtr color_img_ptr = it->first;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg = it->second;

    rclcpp::Time timestamp = color_img_ptr->header.stamp;

    // Process this image in the queue
    if (canTransform(color_img_ptr->header)) {
      images_to_process.push_back(
        std::make_pair(color_img_ptr, camera_info_msg));
    } else {
      continue;
    }

    // If we processed this frame, keep track of that fact so we can delete it
    // at the end.
    if (it_first_valid == color_image_queue_.end()) {
      it_first_valid = it;
    }
    if (it_last_valid <= it) {
      it_last_valid = it;
    }
  }

  // Now we have 2 iterators pointing to what we want to delete.
  if (it_first_valid != color_image_queue_.end()) {
    // Actually erase from the beginning of the queue.
    color_image_queue_.erase(color_image_queue_.begin(), ++it_last_valid);
  }
  lock.unlock();

  // Now we actually process the color images.
  if (images_to_process.empty()) {
    return;
  }

  rclcpp::Time last_timestamp;
  for (auto image_pair : images_to_process) {
    // Cache clock_now.
    rclcpp::Time clock_now = image_pair.first->header.stamp;

    if (max_color_update_hz_ > 0.0f &&
      (clock_now - last_color_update_time_).seconds() <
      1.0f / max_color_update_hz_)
    {
      // Skip integrating this.
      continue;
    }
    last_color_update_time_ = clock_now;

    processColorImage(image_pair.first, image_pair.second);
  }
}

bool NvbloxNode::canTransform(const std_msgs::msg::Header & header)
{
  Transform T_S_C;
  return transformer_.lookupTransformToGlobalFrame(
    header.frame_id,
    header.stamp, &T_S_C);
}

bool NvbloxNode::processDepthImage(
  sensor_msgs::msg::Image::ConstSharedPtr & depth_img_ptr,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  timing::Timer ros_tsdf_timer("ros/tsdf");
  timing::Timer transform_timer("ros/tsdf/transform");
  // Get the TF for this image.
  Transform T_S_C;
  std::string target_frame = depth_img_ptr->header.frame_id;

  if (!transformer_.lookupTransformToGlobalFrame(
      target_frame, depth_img_ptr->header.stamp, &T_S_C))
  {
    return false;
  }
  transform_timer.Stop();

  timing::Timer conversions_timer("ros/tsdf/conversions");
  // Convert camera info message to camera object.
  Camera camera = converter_.cameraFromMessage(*camera_info_msg);

  // Convert the depth image.
  if (!converter_.depthImageFromImageMessage(depth_img_ptr, &depth_image_)) {
    RCLCPP_ERROR(get_logger(), "Failed to transform depth image.");
    return false;
  }
  conversions_timer.Stop();

  // Integrate
  timing::Timer integration_timer("ros/tsdf/integrate");
  mapper_->integrateDepth(depth_image_, T_S_C, camera);
  integration_timer.Stop();
  return true;
}

bool NvbloxNode::processColorImage(
  sensor_msgs::msg::Image::ConstSharedPtr & color_img_ptr,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  timing::Timer ros_color_timer("ros/color");
  timing::Timer transform_timer("ros/color/transform");

  // Get the TF for this image.
  const std::string target_frame = color_img_ptr->header.frame_id;
  Transform T_S_C;

  if (!transformer_.lookupTransformToGlobalFrame(
      target_frame, color_img_ptr->header.stamp, &T_S_C))
  {
    return false;
  }

  transform_timer.Stop();

  timing::Timer color_convert_timer("ros/color/conversion");

  // Convert camera info message to camera object.
  Camera camera = converter_.cameraFromMessage(*camera_info_msg);

  // Convert the color image.
  if (!converter_.colorImageFromImageMessage(color_img_ptr, &color_image_)) {
    RCLCPP_ERROR(get_logger(), "Failed to transform color image.");
    return false;
  }
  color_convert_timer.Stop();

  // Integrate.
  timing::Timer color_integrate_timer("ros/color/integrate");
  mapper_->integrateColor(color_image_, T_S_C, camera);
  color_integrate_timer.Stop();
  return true;
}

bool NvbloxNode::processLidarPointcloud(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_ptr)
{
  timing::Timer ros_lidar_timer("ros/lidar");
  timing::Timer transform_timer("ros/lidar/transform");

  // Get the TF for this image.
  const std::string target_frame = pointcloud_ptr->header.frame_id;
  Transform T_S_C;

  if (!transformer_.lookupTransformToGlobalFrame(
      target_frame, pointcloud_ptr->header.stamp, &T_S_C))
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
  if (!converter_.checkLidarPointcloud(pointcloud_ptr, lidar)) {
    RCLCPP_ERROR_ONCE(
      get_logger(),
      "LiDAR intrinsics are inconsistent with the received "
      "pointcloud. Failing integration.");
    return true;
  }

  timing::Timer lidar_conversion_timer("ros/lidar/conversion");
  converter_.depthImageFromPointcloudGPU(
    pointcloud_ptr, lidar,
    &pointcloud_image_);
  lidar_conversion_timer.Stop();

  timing::Timer lidar_integration_timer("ros/lidar/integration");

  mapper_->integrateLidarDepth(pointcloud_image_, T_S_C, lidar);
  lidar_integration_timer.Stop();

  return true;
}

void NvbloxNode::updateEsdf(const rclcpp::Time & timestamp)
{
  timing::Timer ros_esdf_timer("ros/esdf");

  timing::Timer esdf_integration_timer("ros/esdf/integrate");

  std::vector<Index3D> updated_blocks;
  if (esdf_2d_) {
    updated_blocks =
      mapper_->updateEsdfSlice(min_height_, max_height_, slice_height_);
  } else {
    updated_blocks = mapper_->updateEsdf();
  }

  esdf_integration_timer.Stop();

  if (updated_blocks.empty()) {
    return;
  }

  timing::Timer esdf_output_timer("ros/esdf/output");

  if (pointcloud_publisher_->get_subscription_count() > 0) {
    timing::Timer output_pointcloud_timer("ros/esdf/output/pointcloud");

    // Output the ESDF. Let's just do the full thing for now.
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    // AABB of a certain slice height.
    AxisAlignedBoundingBox aabb(Vector3f(
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        slice_height_ - voxel_size_ / 2.0f),
      Vector3f(
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        slice_height_ + voxel_size_ / 2.0f));

    converter_.pointcloudFromLayerInAABB(
      mapper_->esdf_layer(), aabb,
      &pointcloud_msg);

    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = timestamp;
    pointcloud_publisher_->publish(pointcloud_msg);

    output_pointcloud_timer.Stop();
  }

  // Also publish the map slice.
  if (distance_slice_ && map_slice_publisher_->get_subscription_count() > 0) {
    timing::Timer output_map_slice_timer("ros/esdf/output/map_slice");

    nvblox_msgs::msg::DistanceMapSlice map_slice;

    converter_.distanceMapSliceFromLayer(
      mapper_->esdf_layer(), slice_height_,
      &map_slice);
    map_slice.header.frame_id = global_frame_;
    map_slice.header.stamp = timestamp;
    map_slice_publisher_->publish(map_slice);
  }
}

void NvbloxNode::updateMesh(const rclcpp::Time & timestamp)
{
  timing::Timer ros_mesh_timer("ros/mesh");

  timing::Timer mesh_integration_timer("ros/mesh/integrate_and_color");
  const std::vector<Index3D> mesh_updated_list = mapper_->updateMesh();
  mesh_integration_timer.Stop();

  // In the case that some mesh blocks have been re-added after deletion, remove them from the
  // deleted list.
  for (const Index3D & idx : mesh_updated_list) {
    mesh_blocks_deleted_.erase(idx);
  }
  // Make a list to be published to rviz of blocks to be removed from the viz
  const std::vector<Index3D> mesh_blocks_to_delete(
    mesh_blocks_deleted_.begin(), mesh_blocks_deleted_.end());
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

      converter_.meshMessageFromMeshBlocks(
        mapper_->mesh_layer(), mapper_->mesh_layer().getAllBlockIndices(),
        &mesh_msg);
      mesh_msg.clear = true;
      should_publish = true;
    } else {
      converter_.meshMessageFromMeshBlocks(
        mapper_->mesh_layer(),
        mesh_updated_list, &mesh_msg, mesh_blocks_to_delete);
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
    converter_.markerMessageFromMeshLayer(
      mapper_->mesh_layer(), global_frame_,
      &marker_msg);

    mesh_marker_publisher_->publish(marker_msg);
  }

  mesh_output_timer.Stop();
}

void NvbloxNode::clearMapOutsideOfRadius(
  const std::string & target_frame_id,
  const rclcpp::Time & timestamp)
{
  if (map_clearing_radius_m_ > 0.0f) {
    timing::Timer("ros/clear_outside_radius");
    // We use the most recent but processed transform to define the center.
    Transform T_S_C;
    if (transformer_.lookupTransformToGlobalFrame(target_frame_id, timestamp, &T_S_C)) {
      std::vector<Index3D> blocks_cleared = mapper_->clearOutsideRadius(
        T_S_C.translation(), map_clearing_radius_m_);
      // We keep track of the deleted blocks for publishing later.
      mesh_blocks_deleted_.insert(blocks_cleared.begin(), blocks_cleared.end());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Could not get the transform in order clear the map. This should not be possible.");
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

  response->success = mapper_->saveMap(filename);
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

  response->success = mapper_->loadMap(filename);
  if (response->success) {
    RCLCPP_INFO_STREAM(get_logger(), "Loaded map to file from " << filename);
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Failed to load map file from " << filename);
  }
}

}  // namespace nvblox
