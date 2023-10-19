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

#ifndef NVBLOX_ROS__IMPL__NVBLOX_NODE_IMPL_HPP_
#define NVBLOX_ROS__IMPL__NVBLOX_NODE_IMPL_HPP_

#include <nvblox/utils/timing.h>

#include <deque>
#include <string>
#include <vector>

namespace nvblox
{

template<typename QueuedType>
void NvbloxNode::processMessageQueue(
  std::deque<QueuedType> * queue_ptr, std::mutex * queue_mutex_ptr,
  MessageReadyCallback<QueuedType> message_ready_check,
  ProcessMessageCallback<QueuedType> callback)
{
  timing::Timer ros_total_timer("ros/total");

  // Copy over all the pointers we actually want to process here.
  std::vector<QueuedType> items_to_process;

  std::unique_lock<std::mutex> lock(*queue_mutex_ptr);

  if (queue_ptr->empty()) {
    lock.unlock();
    return;
  }

  auto it_first_valid = queue_ptr->end();
  auto it_last_valid = queue_ptr->begin();

  for (auto it = queue_ptr->begin(); it != queue_ptr->end(); it++) {
    // Process this image in the queue
    if (message_ready_check(*it)) {
      items_to_process.push_back(*it);
    } else {
      continue;
    }

    // If we processed this frame, keep track of that fact so we can delete it
    // at the end.
    if (it_first_valid == queue_ptr->end()) {
      it_first_valid = it;
    }
    if (it_last_valid <= it) {
      it_last_valid = it;
    }
  }

  // Now we have 2 iterators pointing to what we want to delete.
  if (it_first_valid != queue_ptr->end()) {
    // Actually erase from the beginning of the queue.
    queue_ptr->erase(queue_ptr->begin(), ++it_last_valid);
    // Warn user if we're loosing messages unprocessed
    const int num_messages_deleted = it_last_valid - queue_ptr->begin();
    const int num_messages_processed = items_to_process.size();
    if (num_messages_deleted > num_messages_processed) {
      const int num_messages_lost = num_messages_deleted - num_messages_processed;
      constexpr int kLostMessagesPublishPeriodMs = 1000;
      auto & clk = *get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), clk, kLostMessagesPublishPeriodMs,
        "Deleted " << num_messages_lost << "because we could not interpolate transforms.");
    }
  }
  lock.unlock();

  // Process everything that was found to be ready
  if (items_to_process.empty()) {
    return;
  }
  rclcpp::Time last_timestamp;
  for (auto image_pair : items_to_process) {
    callback(image_pair);
  }

  // nvblox statistics
  constexpr int kPublishPeriodMs = 10000;
  auto & clk = *get_clock();
  RCLCPP_INFO_STREAM_THROTTLE(
    get_logger(), clk, kPublishPeriodMs,
    "Timing statistics: \n" <<
      nvblox::timing::Timing::Print());
}

template<typename MessageType>
void NvbloxNode::pushMessageOntoQueue(
  MessageType message,
  std::deque<MessageType> * queue_ptr,
  std::mutex * queue_mutex_ptr)
{
  // Push it into the queue.
  timing::Timer ros_total_timer("ros/total");
  {
    const std::lock_guard<std::mutex> lock(*queue_mutex_ptr);
    queue_ptr->emplace_back(message);
  }
}

template<typename MessageType>
void NvbloxNode::limitQueueSizeByDeletingOldestMessages(
  const int max_num_messages, const std::string & queue_name,
  std::deque<MessageType> * queue_ptr, std::mutex * queue_mutex_ptr)
{
  // Delete extra elements in the queue.
  timing::Timer ros_total_timer("ros/total");
  const std::lock_guard<std::mutex> lock(*queue_mutex_ptr);
  if (queue_ptr->size() > max_num_messages) {
    const int num_elements_to_delete = queue_ptr->size() - max_num_messages;
    queue_ptr->erase(
      queue_ptr->begin(),
      queue_ptr->begin() + num_elements_to_delete);
    constexpr int kPublishPeriodMs = 1000;
    auto & clk = *get_clock();
    RCLCPP_INFO_STREAM_THROTTLE(
      get_logger(), clk, kPublishPeriodMs,
      queue_name << " queue was longer than " << max_num_messages <<
        " deleted " << num_elements_to_delete << " messages.");
  }
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
