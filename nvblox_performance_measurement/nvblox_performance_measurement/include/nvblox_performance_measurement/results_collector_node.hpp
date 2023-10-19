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

#ifndef NVBLOX_PERFORMANCE_MEASUREMENT__RESULTS_COLLECTOR_NODE_HPP_
#define NVBLOX_PERFORMANCE_MEASUREMENT__RESULTS_COLLECTOR_NODE_HPP_

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

#include <nvblox_performance_measurement_msgs/srv/file_path.hpp>
#include <nvblox_performance_measurement_msgs/srv/get_results.hpp>

#include "nvblox_performance_measurement/message_stamp_recorder.hpp"

namespace nvblox
{

class ResultsCollectorNode : public rclcpp::Node
{
public:
  ResultsCollectorNode();

  // Send results back to a requester
  void getResultsCallback(
    const std::shared_ptr<
      nvblox_performance_measurement_msgs::srv::GetResults::Request>
    request,
    std::shared_ptr<
      nvblox_performance_measurement_msgs::srv::GetResults::Response>
    response) const;

  // CPU usage callback
  void cpuMessageCallback(const std_msgs::msg::Float32::ConstSharedPtr msg_ptr);
  void gpuMessageCallback(const std_msgs::msg::Float32::ConstSharedPtr msg_ptr);

  // Network performance callback
  void NetMeanIoUMessageCallback(const std_msgs::msg::Float32::ConstSharedPtr msg_ptr);

  // Nvblox timers callback
  void timersMessageCallback(const std_msgs::msg::String::ConstSharedPtr msg_ptr);

private:
  // Arrival recorders for each topic of interest
  std::vector<std::unique_ptr<MessageStampRecorderInterface>> recorders_;

  // Samples of CPU/GPU percentage
  std::vector<float> cpu_percentages_;
  std::vector<float> gpu_percentages_;

  // Samples of network performance measurements
  std::vector<float> network_mean_iou_percentages_;

  // The last timing results received
  std::string nvblox_timers_string_;

  // Services
  rclcpp::Service<nvblox_performance_measurement_msgs::srv::GetResults>::
  SharedPtr get_results_service_;

  // Subscription to CPU/GPU load
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cpu_percentage_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gpu_percentage_sub_;

  // Subscribe to network mean IoU
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr network_mean_iou_percentage_sub_;

  // Subscription to the nvblox internal timing results
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr timers_sub_;

  // Default topic names
  // NOTE: These are also the names against which the stamps are stored such
  // that they can be interpreted consistently in spite of remapping.
  const std::string color_image_topic_name_ = "color/image";
  const std::string depth_image_topic_name_ = "depth/image";
  const std::string semantic_image_topic_name_ = "/unet/raw_segmentation_mask";
  const std::string pointcloud_topic_name_ = "pointcloud";
  const std::string color_image_processed_topic_name_ = "/nvblox_human_node/color_processed";
  const std::string depth_image_processed_topic_name_ = "/nvblox_human_node/depth_processed";
  const std::string pointcloud_processed_topic_name_ = "/nvblox_human_node/pointcloud_processed";
  const std::string base_slice_topic_name_ = "/nvblox_human_node/static_map_slice";
  const std::string human_slice_topic_name_ = "/nvblox_human_node/dynamic_map_slice";
  const std::string mesh_processed_name_ = "/nvblox_human_node/mesh_processed";
  const std::string cpu_topic_name_ = "cpu_percentage_node/cpu_percent";
  const std::string gpu_topic_name_ = "gpu_percentage_node/gpu_percent";
  const std::string network_mean_iou_topic_name_ = "/network_performance_node/iou";

};

}  // namespace nvblox

#endif  // NVBLOX_PERFORMANCE_MEASUREMENT__RESULTS_COLLECTOR_NODE_HPP_
