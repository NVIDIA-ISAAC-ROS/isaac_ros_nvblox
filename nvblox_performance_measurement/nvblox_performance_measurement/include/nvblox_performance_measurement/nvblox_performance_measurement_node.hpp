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

#ifndef NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_NODE_HPP_
#define NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_NODE_HPP_

#include <std_msgs/msg/string.hpp>

#include <nvblox_ros/nvblox_node.hpp>

#include <nvblox_performance_measurement_msgs/msg/frame_processed.hpp>

namespace nvblox
{

class NvbloxPerformanceMeasurementNode : public NvbloxNode
{
public:
  NvbloxPerformanceMeasurementNode();
  virtual ~NvbloxPerformanceMeasurementNode() = default;

  // Callback functions. These just stick images in a queue.
  // Process a single images
  virtual bool processDepthImage(
    sensor_msgs::msg::Image::ConstSharedPtr & depth_img_ptr,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg) override;
  virtual bool processColorImage(
    sensor_msgs::msg::Image::ConstSharedPtr & color_img_ptr,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg) override;
  virtual bool processLidarPointcloud(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_ptr) override;

  virtual void processMesh() override;

  void publishTimersCallback();

private:
  // Parameters
  float timer_publish_rate_hz_ = 1.0;

  // Publishers
  rclcpp::Publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed>::SharedPtr
    depth_processed_publisher_;
  rclcpp::Publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed>::SharedPtr
    color_processed_publisher_;
  rclcpp::Publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed>::SharedPtr
    pointcloud_processed_publisher_;
  rclcpp::Publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed>::SharedPtr
    mesh_processed_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timers_publisher_;

  // Timers
  rclcpp::TimerBase::SharedPtr timers_publishing_timer_;
};

}  // namespace nvblox

#endif  // NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_NODE_HPP_
