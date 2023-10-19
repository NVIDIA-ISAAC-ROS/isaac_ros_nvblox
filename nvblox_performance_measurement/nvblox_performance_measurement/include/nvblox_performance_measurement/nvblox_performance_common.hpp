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

#ifndef NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_COMMON_HPP_
#define NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_COMMON_HPP_

#include <functional>

#include <rclcpp/publisher.hpp>

#include <nvblox_performance_measurement_msgs/msg/frame_processed.hpp>

namespace nvblox
{

/// The type of a function that processes a message.
template<typename MessageType>
using ProcessMessageFunction = std::function<bool (const MessageType &)>;

/// The type of a function that performs some function.
using ProcessFunction = std::function<void (void)>;

/// The type of a function which extracts a header from a message.
template<typename MessageType>
using GetHeaderFunction =
  std::function<std_msgs::msg::Header(const MessageType &)>;

/// @brief Calls a ProcessMessage function and emits a header if successful.
/// @tparam MessageType The type of the message to process
/// @param process_function The function to call.
/// @param message The message to process.
/// @param get_header_function A function to extract a header from a message.
/// @param processed_publisher_ptr The publisher on which to emit the header.
/// @return True if the process message function succeeded.
template<typename MessageType>
bool callProcessMessageAndEmit(
  const ProcessMessageFunction<MessageType> & process_function,
  const MessageType & message,
  const GetHeaderFunction<MessageType> & get_header_function,
  rclcpp::Publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed> *
  processed_publisher_ptr);

/// @brief Calls a function and emits a header.
/// @param process_function The function to call.
/// @param stamp The timestamp of the header to be emitted.
/// @param processed_publisher_ptr The publisher on which to emit the header.
void callAndEmit(
  const ProcessFunction & process_function, const rclcpp::Time & stamp,
  rclcpp::Publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed> *
  processed_publisher_ptr);

}  // namespace nvblox

#endif  // NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_COMMON_HPP_

#include "nvblox_performance_measurement/impl/nvblox_performance_common_impl.hpp"
