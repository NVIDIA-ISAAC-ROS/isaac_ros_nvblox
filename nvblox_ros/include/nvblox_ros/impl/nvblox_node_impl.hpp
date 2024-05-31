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

#ifndef NVBLOX_ROS__IMPL__NVBLOX_NODE_IMPL_HPP_
#define NVBLOX_ROS__IMPL__NVBLOX_NODE_IMPL_HPP_

#include <nvblox/utils/timing.h>
#include <nvblox/utils/rates.h>
#include <nvblox/utils/delays.h>

#include <list>
#include <string>
#include <vector>

namespace nvblox
{

template<typename QueuedType>
void NvbloxNode::processMessageQueue(
  std::list<QueuedType> * queue_ptr, std::mutex * queue_mutex_ptr,
  MessageReadyCallback<QueuedType> message_ready_check,
  ProcessMessageCallback<QueuedType> callback)
{
  timing::Timer ros_total_timer("ros/process_message_queue");

  // Iterate over all items in the queue and delete the ones processed.
  std::unique_lock<std::mutex> lock(*queue_mutex_ptr);
  auto it = queue_ptr->begin();
  while (it != queue_ptr->end()) {
    auto next = std::next(it);
    if (message_ready_check(*it)) {
      callback(*it);
      queue_ptr->erase(it);
    }
    it = next;
  }
  lock.unlock();


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
}

template<typename MessageType>
void NvbloxNode::pushMessageOntoQueue(
  const std::string & queue_name,
  MessageType message,
  std::list<MessageType> * queue_ptr,
  std::mutex * queue_mutex_ptr)
{
  const std::lock_guard<std::mutex> lock(*queue_mutex_ptr);
  // Size should never grow larger than allowed if this is the only place we modify the queue.
  CHECK(queue_ptr->size() <= static_cast<size_t>(params_.maximum_sensor_message_queue_length));

  // Make room for the new message if queue is full
  if (queue_ptr->size() == static_cast<size_t>(params_.maximum_sensor_message_queue_length)) {
    queue_ptr->pop_front();
    number_of_dropped_messages_[queue_name]++;
    constexpr int kPublishPeriodMs = 1000;
    auto & clk = *get_clock();
    RCLCPP_INFO_STREAM_THROTTLE(
      get_logger(), clk, kPublishPeriodMs,
      "Dropped an item from: " << queue_name << ". Size of queue: " << queue_ptr->size() <<
        ". Total number of dropped messages is: " <<
        number_of_dropped_messages_[queue_name]);
  }
  queue_ptr->emplace_back(message);
}

template<typename MessageType>
void NvbloxNode::printMessageArrivalStatistics(
  const MessageType & message, const std::string & output_prefix,
  libstatistics_collector::topic_statistics_collector::
  ReceivedMessagePeriodCollector<MessageType> * statistics_collector)
{
  // Calculate statistics
  statistics_collector->OnMessageReceived(
    message,
    get_clock()->now().nanoseconds());
  // Print statistics
  constexpr int kPublishPeriodMs = 10000;
  auto & clk = *get_clock();
  RCLCPP_INFO_STREAM_THROTTLE(
    get_logger(), clk, kPublishPeriodMs,
    output_prefix << ": \n" <<
      libstatistics_collector::moving_average_statistics::
      StatisticsDataToString(
      statistics_collector->GetStatisticsResults()));
}

}  // namespace nvblox

#endif  // NVBLOX_ROS__IMPL__NVBLOX_NODE_IMPL_HPP_
