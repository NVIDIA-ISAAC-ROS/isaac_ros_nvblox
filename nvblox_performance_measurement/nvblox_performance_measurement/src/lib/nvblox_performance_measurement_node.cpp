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

#include "nvblox_performance_measurement/nvblox_performance_measurement_node.hpp"

#include <nvblox/utils/timing.h>

namespace nvblox
{

NvbloxPerformanceMeasurementNode::NvbloxPerformanceMeasurementNode()
: NvbloxNode()
{
  // Parameters
  timer_publish_rate_hz_ =
    declare_parameter<float>("timer_publish_rate_hz", timer_publish_rate_hz_);

  // Publishers
  depth_processed_publisher_ =
    create_publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed>(
    "~/depth_processed", 10);
  color_processed_publisher_ =
    create_publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed>(
    "~/color_processed", 10);
  pointcloud_processed_publisher_ =
    create_publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed>(
    "~/pointcloud_processed", 10);
  mesh_processed_publisher_ =
    create_publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed>(
    "~/mesh_processed", 10);
  timers_publisher_ =
    create_publisher<std_msgs::msg::String>(
    "~/timers", 10);

  // Timers
  timers_publishing_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / timer_publish_rate_hz_),
    std::bind(&NvbloxPerformanceMeasurementNode::publishTimersCallback, this));
}

bool NvbloxPerformanceMeasurementNode::processDepthImage(
  sensor_msgs::msg::Image::ConstSharedPtr & depth_img_ptr,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  // Process as declared in NvbloxNode
  const bool success =
    NvbloxNode::processDepthImage(depth_img_ptr, camera_info_msg);
  // Indicate success to the outside world
  if (success) {
    nvblox_performance_measurement_msgs::msg::FrameProcessed msg;
    msg.header = depth_img_ptr->header;
    depth_processed_publisher_->publish(msg);
  }
  return success;
}

bool NvbloxPerformanceMeasurementNode::processColorImage(
  sensor_msgs::msg::Image::ConstSharedPtr & color_img_ptr,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  // Process as declared in NvbloxNode
  const bool success =
    NvbloxNode::processColorImage(color_img_ptr, camera_info_msg);
  // Indicate success to the outside world
  if (success) {
    nvblox_performance_measurement_msgs::msg::FrameProcessed msg;
    msg.header = color_img_ptr->header;
    color_processed_publisher_->publish(msg);
  }
  return success;
}

bool NvbloxPerformanceMeasurementNode::processLidarPointcloud(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_ptr)
{
  // Process as declared in NvbloxNode
  const bool success =
    NvbloxNode::processLidarPointcloud(pointcloud_ptr);
  // Indicate success to the outside world
  if (success) {
    nvblox_performance_measurement_msgs::msg::FrameProcessed msg;
    msg.header = pointcloud_ptr->header;
    pointcloud_processed_publisher_->publish(msg);
  }
  return success;
}

void NvbloxPerformanceMeasurementNode::processMesh()
{
  // Process as declared in NvbloxNode
  NvbloxNode::processMesh();
  // Indicate we updated the mesh
  nvblox_performance_measurement_msgs::msg::FrameProcessed msg;
  msg.header.stamp = get_clock()->now();
  mesh_processed_publisher_->publish(msg);
}


void NvbloxPerformanceMeasurementNode::publishTimersCallback()
{
  std_msgs::msg::String msg;
  msg.data = nvblox::timing::Timing::Print();
  timers_publisher_->publish(msg);
}

}  // namespace nvblox
