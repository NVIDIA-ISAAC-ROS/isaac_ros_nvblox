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

  // Check if a valid mapping typ was selected
  if (!isHumanMapping(mapping_type_)) {
    RCLCPP_FATAL_STREAM(
      get_logger(),
      "Invalid option. The nvblox human node only does human mapping. "
      "Use the basic nvblox node for static or dynamic mapping. Exiting nvblox.");
    exit(1);
  }

  // Get parameters specific to the human node.
  getParameters();

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
  human_debug_publish_rate_hz_ = declare_parameter<float>(
    "human_debug_publish_rate_hz", human_debug_publish_rate_hz_);
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
  segmentation_camera_info_sub_.subscribe(
    this, "mask/camera_info",
    parseQosString(color_qos_str_));

  if (use_depth_) {
    // Unsubscribe from the depth topic in nvblox_node
    timesync_depth_.reset();
    // Subscribe to depth + mask + cam_infos
    timesync_depth_mask_ = std::make_shared<
      message_filters::Synchronizer<mask_approximate_time_policy_t>>(
      mask_approximate_time_policy_t(kQueueSize), depth_sub_,
      depth_camera_info_sub_, segmentation_mask_sub_,
      segmentation_camera_info_sub_);
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
    // We use exact time sync here
    // because color image and its corresponding semantic prediction have the same timestamp
    timesync_color_mask_ = std::make_shared<
      message_filters::Synchronizer<mask_exact_time_policy_t>>(
      mask_exact_time_policy_t(kQueueSize), color_sub_,
      color_camera_info_sub_, segmentation_mask_sub_,
      segmentation_camera_info_sub_);
    timesync_color_mask_->registerCallback(
      std::bind(
        &NvbloxHumanNode::colorPlusMaskImageCallback, this,
        std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4));
  }
}

void NvbloxHumanNode::advertiseTopics()
{
  // Add additional debug output for human mapping
  human_voxels_publisher_ =
    create_publisher<visualization_msgs::msg::Marker>("~/human_voxels", 1);
  color_frame_overlay_publisher_ =
    create_publisher<sensor_msgs::msg::Image>("~/dynamic_color_frame_overlay", 1);
}

void NvbloxHumanNode::setupTimers()
{
  human_debug_publish_timer__ = create_wall_timer(
    std::chrono::duration<double>(1.0 / human_debug_publish_rate_hz_),
    std::bind(&NvbloxHumanNode::publishHumanDebugOutput, this), group_processing_);
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
  if (dynamic_depth_frame_overlay_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::Image img_msg;
    const ColorImage & depth_overlay =
      multi_mapper_->getLastDepthFrameMaskOverlay();
    conversions::imageMessageFromColorImage(
      depth_overlay, depth_img_frame,
      &img_msg);
    dynamic_depth_frame_overlay_publisher_->publish(img_msg);
  }
  overlay_timer.Stop();

  // Publish back projected depth image for debugging
  timing::Timer back_projected_depth_timer("ros/depth/output/back_projected_depth");
  if (back_projected_depth_publisher_->get_subscription_count() > 0) {
    publishBackProjectedDepth(depth_camera_, T_L_C_depth_);
  }
  back_projected_depth_timer.Stop();

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

void NvbloxHumanNode::publishHumanDebugOutput()
{
  timing::Timer ros_total_timer("ros/total");
  timing::Timer ros_human_total_timer("ros/humans");
  timing::Timer ros_human_debug_timer("ros/humans/output/debug");

  // Get a human pointcloud
  if (dynamic_points_publisher_->get_subscription_count() +
    human_voxels_publisher_->get_subscription_count() >
    0)
  {
    // Grab the human only image.
    const DepthImage & depth_image_only_humans =
      multi_mapper_->getLastDepthFrameMasked();
    // Back project
    image_back_projector_.backProjectOnGPU(
      depth_image_only_humans, depth_camera_, &human_pointcloud_C_device_,
      dynamic_mapper_->occupancy_integrator().max_integration_distance_m());
    transformPointcloudOnGPU(
      T_L_C_depth_, human_pointcloud_C_device_,
      &human_pointcloud_L_device_);
  }

  // Publish the human pointcloud
  if (dynamic_points_publisher_->get_subscription_count() > 0) {
    // Back-project human depth image to pointcloud and publish.
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pointcloud_converter_.pointcloudMsgFromPointcloud(
      human_pointcloud_L_device_, &pointcloud_msg);
    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = get_clock()->now();
    dynamic_points_publisher_->publish(pointcloud_msg);
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

}  // namespace nvblox

// Register the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvblox::NvbloxHumanNode)
