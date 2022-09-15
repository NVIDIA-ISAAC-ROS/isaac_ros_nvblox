/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

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

  // Nvblox timers callback
  void timersMessageCallback(const std_msgs::msg::String::ConstSharedPtr msg_ptr);

private:
  // Arrival recorders for each topic of interest
  std::vector<std::unique_ptr<MessageStampRecorderInterface>> recorders_;

  // Samples of CPU/GPU percentage
  std::vector<float> cpu_percentages_;
  std::vector<float> gpu_percentages_;

  // The last timing results received
  std::string nvblox_timers_string_;

  // Services
  rclcpp::Service<nvblox_performance_measurement_msgs::srv::GetResults>::
  SharedPtr get_results_service_;

  // Subscription to CPU/GPU load
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cpu_percentage_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gpu_percentage_sub_;

  // Subscription to the nvblox internal timing results
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr timers_sub_;

  // Default topic names
  // NOTE: These are also the names against which the stamps are stored such
  // that they can be interpreted consistently in spite of remapping.
  const std::string color_image_topic_name_ = "color/image";
  const std::string depth_image_topic_name_ = "depth/image";
  const std::string pointcloud_topic_name_ = "pointcloud";
  const std::string color_image_processed_topic_name_ = "/nvblox_node/color_processed";
  const std::string depth_image_processed_topic_name_ = "/nvblox_node/depth_processed";
  const std::string pointcloud_processed_topic_name_ = "/nvblox_node/pointcloud_processed";
  const std::string slice_topic_name_ = "/nvblox_node/map_slice";
  const std::string mesh_processed_name_ = "/nvblox_node/mesh_processed";
  const std::string cpu_topic_name_ = "cpu_percentage_node/cpu_percent";
  const std::string gpu_topic_name_ = "gpu_percentage_node/gpu_percent";

};

}  // namespace nvblox

#endif  // NVBLOX_PERFORMANCE_MEASUREMENT__RESULTS_COLLECTOR_NODE_HPP_
