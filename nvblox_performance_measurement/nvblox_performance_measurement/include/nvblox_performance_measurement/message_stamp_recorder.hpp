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

#ifndef NVBLOX_PERFORMANCE_MEASUREMENT__MESSAGE_STAMP_RECORDER_HPP_
#define NVBLOX_PERFORMANCE_MEASUREMENT__MESSAGE_STAMP_RECORDER_HPP_

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <nvblox_msgs/msg/distance_map_slice.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/empty.hpp>

using std::placeholders::_1;

namespace nvblox
{

class MessageStampRecorderInterface
{
public:
  MessageStampRecorderInterface(
    rclcpp::Node * node_ptr, const std::string topic_name = std::string())
  : node_ptr_(node_ptr), topic_name_(topic_name) {}

  const std::vector<rclcpp::Time> & received_stamps() const;
  virtual std::string topic_name() const = 0;

protected:
  // The node associated with this recorder
  rclcpp::Node * node_ptr_;

  // Topic name
  std::string topic_name_;

  // Timestamp array
  std::vector<rclcpp::Time> received_stamps_;
};

template<typename MessageType>
class MessageStampRecorder : public MessageStampRecorderInterface
{
public:
  MessageStampRecorder(
    rclcpp::Node * node_ptr, const std::string & topic_name,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_default);

  virtual std::string topic_name() const;

  void messageCallback(const typename MessageType::ConstSharedPtr msg_ptr);

private:
  // Image subscriber
  typename rclcpp::Subscription<MessageType>::SharedPtr sub_;
};

}  // namespace nvblox

#include "nvblox_performance_measurement/impl/message_stamp_recorder_impl.hpp"

#endif  // NVBLOX_PERFORMANCE_MEASUREMENT__MESSAGE_STAMP_RECORDER_HPP_
