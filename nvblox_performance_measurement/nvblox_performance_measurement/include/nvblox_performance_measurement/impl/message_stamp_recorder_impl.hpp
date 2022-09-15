/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

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
