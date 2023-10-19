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

#ifndef NVBLOX_PERFORMANCE_MEASUREMENT__MESSAGE_STAMP_RECORDER_IMPL_HPP_
#define NVBLOX_PERFORMANCE_MEASUREMENT__MESSAGE_STAMP_RECORDER_IMPL_HPP_

#include <fstream>

namespace nvblox
{

template<typename MessageType>
MessageStampRecorder<MessageType>::MessageStampRecorder(
  rclcpp::Node * node_ptr, const std::string & topic_name,
  const rmw_qos_profile_t & qos_profile)
: MessageStampRecorderInterface(node_ptr, topic_name)
{
  // Setting up the profile
  constexpr size_t kQueueSize = 10;
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(kQueueSize), qos_profile);
  // Subscribing
  sub_ = node_ptr_->create_subscription<MessageType>(
    topic_name, qos,
    std::bind(&MessageStampRecorder::messageCallback, this, _1));
}

template<typename MessageType>
std::string MessageStampRecorder<MessageType>::topic_name() const
{
  if (topic_name_.empty()) {
    return sub_->get_topic_name();
  }
  return topic_name_;
}

template<typename MessageType>
void MessageStampRecorder<MessageType>::messageCallback(
  const typename MessageType::ConstSharedPtr msg_ptr)
{
  // Debug output
  constexpr int kPublishPeriodMs = 1000;
  auto & clk = *node_ptr_->get_clock();
  RCLCPP_INFO_THROTTLE(
    node_ptr_->get_logger(), clk, kPublishPeriodMs,
    "Message received.\n");
  RCLCPP_INFO_THROTTLE(
    node_ptr_->get_logger(), clk, kPublishPeriodMs,
    "Timestamps size: " + received_stamps_.size());

  // Record the timestamp
  received_stamps_.push_back(msg_ptr->header.stamp);
}

}  // namespace nvblox

#endif  // NVBLOX_PERFORMANCE_MEASUREMENT__MESSAGE_STAMP_RECORDER_IMPL_HPP_
