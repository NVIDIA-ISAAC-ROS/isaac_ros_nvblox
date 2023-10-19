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

#ifndef NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_IMPL_HPP_
#define NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_IMPL_HPP_

#include "nvblox_performance_measurement/nvblox_performance_common.hpp"

namespace nvblox
{


template<typename MessageType>
bool callProcessMessageAndEmit(
  const ProcessMessageFunction<MessageType> & process_function,   // NOLINT
  const MessageType & message,                                    // NOLINT
  const GetHeaderFunction<MessageType> & get_header_function,     // NOLINT
  rclcpp::Publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed> *
  processed_publisher_ptr)
{
  // Process using underlying method
  const bool success = process_function(message);
  // Indicate success to the outside world
  if (success) {
    nvblox_performance_measurement_msgs::msg::FrameProcessed
      frame_processed_msg;
    frame_processed_msg.header = get_header_function(message);
    processed_publisher_ptr->publish(frame_processed_msg);
  }
  return success;
}

void callAndEmit(
  const ProcessFunction & process_function,   // NOLINT
  const rclcpp::Time & stamp,                 // NOLINT
  rclcpp::Publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed> *
  processed_publisher_ptr)
{
  // Process using underlying method
  process_function();
  // Indicate success to the outside world
  nvblox_performance_measurement_msgs::msg::FrameProcessed frame_processed_msg;
  frame_processed_msg.header.stamp = stamp;
  processed_publisher_ptr->publish(frame_processed_msg);
}

}  // namespace nvblox

#endif  // NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_IMPL_HPP_
