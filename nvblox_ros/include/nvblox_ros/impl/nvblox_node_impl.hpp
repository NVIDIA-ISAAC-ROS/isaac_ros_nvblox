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

#include <nvblox/utils/delays.h>
#include <nvblox/utils/rates.h>
#include <nvblox/utils/timing.h>

#include <list>
#include <memory>
#include <string>
#include <vector>

namespace nvblox
{

template<typename QueuedType>
void NvbloxNode::processQueue(
  std::unique_ptr<std::list<QueuedType>> & queue_ptr, std::mutex * queue_mutex_ptr,
  ReadyCheckFunctionType<QueuedType> ready_check_function,
  ProcessFunctionType<QueuedType> process_function)
{
  timing::Timer ros_total_timer("ros/process_queue");
  if (queue_ptr->empty()) {
    return;
  }

  // To avoid holding onto the queue lock longer than necessary, we first create copies of the
  // elements in the queue before processing them.
  std::unique_lock<std::mutex> lock(*queue_mutex_ptr);
  std::vector<QueuedType> items_to_process;
  items_to_process.reserve(queue_ptr->size());

  typename std::list<QueuedType>::iterator itr = queue_ptr->begin();
  while (itr != queue_ptr->end()) {
    if (ready_check_function(*itr)) {
      items_to_process.push_back(*itr);
      itr = queue_ptr->erase(itr);
    } else {
      ++itr;
    }
  }
  lock.unlock();

  // Process all items
  for (auto item : items_to_process) {
    process_function(item);
  }
}


template<typename QueuedType>
void NvbloxNode::pushOntoQueue(
  const std::string & queue_name, QueuedType item,
  std::unique_ptr<std::list<QueuedType>> & queue_ptr,
  std::mutex * queue_mutex_ptr)
{
  timing::Timer lock_timer("ros/push_onto_queue");
  std::unique_lock<std::mutex> lock(*queue_mutex_ptr);
  // Size should never grow larger than allowed since this is the only place where we push to the
  // queue.
  CHECK(queue_ptr->size() <= static_cast<size_t>(params_.maximum_input_queue_length));

  queue_ptr->emplace_back(item);
  const int queue_ptr_size = queue_ptr->size();

  // Remove oldest item if queue is full
  if (queue_ptr_size > params_.maximum_input_queue_length) {
    queue_ptr->pop_front();
    lock.unlock();

    // Print some debug info
    if (params_.print_queue_drops_to_console) {
      number_of_dropped_queued_items_[queue_name]++;
      auto & clk = *get_clock();
      RCLCPP_INFO_STREAM_THROTTLE(
        get_logger(), clk, params_.print_statistics_on_console_period_ms,
        "Dropped an item from: " <<
          queue_name << ". Size of queue: " << queue_ptr_size <<
          ". Total number of dropped items is: " <<
          number_of_dropped_queued_items_[queue_name]);
    }
  } else {
    lock.unlock();
  }
}

}  // namespace nvblox

#endif  // NVBLOX_ROS__IMPL__NVBLOX_NODE_IMPL_HPP_
