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

#include "nvblox_performance_measurement/results_collector_node.hpp"

#include <nvblox_ros_common/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nvblox_performance_measurement_msgs/msg/frame_processed.hpp>
#include <nvblox_performance_measurement_msgs/msg/topic_stamps.hpp>

#include <fstream>

#include <iostream>

namespace nvblox
{

ResultsCollectorNode::ResultsCollectorNode()
: rclcpp::Node("results_collector_node")
{
  // Params - Settings for QoS.
  const std::string kDefaultQoS = "SENSOR_DATA";
  std::string color_qos_str =
    declare_parameter<std::string>("color_qos", kDefaultQoS);
  std::string depth_qos_str =
    declare_parameter<std::string>("depth_qos", kDefaultQoS);
  std::string pointcloud_qos_str =
    declare_parameter<std::string>("pointcloud_qos", kDefaultQoS);

  // Subscriptions (through MessageStampRecorder objects)
  recorders_.push_back(
    std::make_unique<MessageStampRecorder<sensor_msgs::msg::Image>>(
      this, color_image_topic_name_, parseQosString(color_qos_str)));
  recorders_.push_back(
    std::make_unique<MessageStampRecorder<sensor_msgs::msg::Image>>(
      this, depth_image_topic_name_, parseQosString(depth_qos_str)));
  recorders_.push_back(
    std::make_unique<MessageStampRecorder<sensor_msgs::msg::PointCloud2>>(
      this, pointcloud_topic_name_, parseQosString(pointcloud_qos_str)));
  // Add subscriber for semantic output
  recorders_.push_back(
    std::make_unique<MessageStampRecorder<sensor_msgs::msg::Image>>(
      this, semantic_image_topic_name_, parseQosString(color_qos_str)));

  recorders_.push_back(
    std::make_unique<
      MessageStampRecorder<nvblox_performance_measurement_msgs::msg::FrameProcessed>>(
      this, color_image_processed_topic_name_));
  recorders_.push_back(
    std::make_unique<
      MessageStampRecorder<nvblox_performance_measurement_msgs::msg::FrameProcessed>>(
      this, depth_image_processed_topic_name_));
  recorders_.push_back(
    std::make_unique<
      MessageStampRecorder<nvblox_performance_measurement_msgs::msg::FrameProcessed>>(
      this, pointcloud_processed_topic_name_));
  recorders_.push_back(
    std::make_unique<
      MessageStampRecorder<nvblox_performance_measurement_msgs::msg::FrameProcessed>>(
      this, mesh_processed_name_));

  recorders_.push_back(
    std::make_unique<
      MessageStampRecorder<nvblox_msgs::msg::DistanceMapSlice>>(
      this, base_slice_topic_name_));

  recorders_.push_back(
    std::make_unique<
      MessageStampRecorder<nvblox_msgs::msg::DistanceMapSlice>>(
      this, human_slice_topic_name_));

  // Subscribing to CPU/GPU usage
  constexpr size_t kQueueSize = 10;
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(kQueueSize), parseQosString(kDefaultQoS));
  cpu_percentage_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    cpu_topic_name_, qos,
    std::bind(&ResultsCollectorNode::cpuMessageCallback, this, _1));
  gpu_percentage_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    gpu_topic_name_, qos,
    std::bind(&ResultsCollectorNode::gpuMessageCallback, this, _1));

  // network performance subscription
  network_mean_iou_percentage_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    network_mean_iou_topic_name_, qos,
    std::bind(&ResultsCollectorNode::NetMeanIoUMessageCallback, this, _1));

  // nvblox timers string subscription
  timers_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/nvblox_human_node/timers", qos,
    std::bind(&ResultsCollectorNode::timersMessageCallback, this, _1));

  // Services
  get_results_service_ =
    create_service<nvblox_performance_measurement_msgs::srv::GetResults>(
    "~/get_results",
    std::bind(
      &ResultsCollectorNode::getResultsCallback, this,
      std::placeholders::_1, std::placeholders::_2));
}

void ResultsCollectorNode::getResultsCallback(
  const std::shared_ptr<nvblox_performance_measurement_msgs::srv::GetResults::Request>,
  std::shared_ptr<nvblox_performance_measurement_msgs::srv::GetResults::Response>
  response) const
{
  // Populate response
  for (const auto & recorder : recorders_) {
    nvblox_performance_measurement_msgs::msg::TopicStamps msg;
    msg.topic_name = recorder->topic_name();
    const auto & stamps = recorder->received_stamps();
    std::for_each(
      stamps.begin(), stamps.end(), [&msg](const auto & stamp) {
        msg.stamps.push_back(stamp.nanoseconds());
      });
    response->topic_stamps.push_back(msg);
  }
  std::for_each(
    cpu_percentages_.begin(), cpu_percentages_.end(),
    [&response](const float percentage) {
      std_msgs::msg::Float32 msg;
      msg.data = percentage;
      response->cpu_percentage_samples.push_back(msg);
    });
  std::for_each(
    gpu_percentages_.begin(), gpu_percentages_.end(),
    [&response](const float percentage) {
      std_msgs::msg::Float32 msg;
      msg.data = percentage;
      response->gpu_percentage_samples.push_back(msg);
    });
  std::for_each(
    network_mean_iou_percentages_.begin(), network_mean_iou_percentages_.end(),
    [&response](const float percentage) {
      std_msgs::msg::Float32 msg;
      msg.data = percentage;
      response->network_mean_iou_percentage_samples.push_back(msg);
    });
  response->timers_string.data = nvblox_timers_string_;
}

void ResultsCollectorNode::cpuMessageCallback(
  const std_msgs::msg::Float32::ConstSharedPtr msg_ptr)
{
  cpu_percentages_.push_back(msg_ptr->data);
}

void ResultsCollectorNode::gpuMessageCallback(
  const std_msgs::msg::Float32::ConstSharedPtr msg_ptr)
{
  gpu_percentages_.push_back(msg_ptr->data);
}

void ResultsCollectorNode::NetMeanIoUMessageCallback(
  const std_msgs::msg::Float32::ConstSharedPtr msg_ptr)
{
  network_mean_iou_percentages_.push_back(msg_ptr->data);
}

void ResultsCollectorNode::timersMessageCallback(
  const std_msgs::msg::String::ConstSharedPtr msg_ptr)
{
  nvblox_timers_string_ = msg_ptr->data;
}

}  // namespace nvblox
