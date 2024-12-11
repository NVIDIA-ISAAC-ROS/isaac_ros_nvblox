// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVBLOX_ROS__ROSBAG_READING_HPP_
#define NVBLOX_ROS__ROSBAG_READING_HPP_

#include <string>

#include "tf2_ros/buffer.h"

#include "rosbag2_cpp/reader.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "nvblox/core/types.h"

namespace nvblox
{
namespace datasets
{
namespace ros
{

/// Deserializes a serialized ROS message
/// @tparam MessageType The ROS message type
/// @param serialized_msg Serialized message.
/// @return Deserialized message.
template<typename MessageType>
MessageType deserializeMessage(
  const rosbag2_storage::SerializedBagMessage & serialized_msg);

/// Creates a rosbag2_cpp::Reader for a single topic.
/// @param rosbag_path Path to the rosbag to read.
/// @param topic_name The name of the topic to read.
/// @param[out] reader_ptr The reader to create.
void createSingleTopicReader(
  const std::string & rosbag_path,
  const std::string & topic_name,
  rosbag2_cpp::Reader * reader_ptr);

/// Converts a ROS timestamp message to nanoseconds.
/// @param time_msg ROS Time message.
/// @return Nanoseconds as an unsigned int.
uint64_t toNanoSeconds(const builtin_interfaces::msg::Time & time_msg);

/// Less-than operator for ROS timestamp messages.
bool operator<(
  const builtin_interfaces::msg::Time & lhs,
  const builtin_interfaces::msg::Time & rhs);

/// Less-than or equal to operator for ROS timestamp messages.
bool operator<=(
  const builtin_interfaces::msg::Time & lhs,
  const builtin_interfaces::msg::Time & rhs);

/// Returns the lowest timestamp among the timestamps in the TF message.
/// @param msg TFMessage message.
/// @return The lowest/earliest timestamp.
builtin_interfaces::msg::Time getLowestStamp(
  const tf2_msgs::msg::TFMessage & msg);

/// Return a timestamp which is earlier that the input timestamp by some
/// specified amount.
/// @param msg The timestamp message.
/// @param diff_s The amount to go earlier by.
/// @return The earlier timestamp.
builtin_interfaces::msg::Time getEarlierTime(
  const builtin_interfaces::msg::Time & msg, const float diff_s);

/// Read all tf static messages from a bag and load them into a tf_buffer.
/// @param rosbag_path The path to the ROS bag.
/// @param tf_buffer_ptr The buffer to load the timestamps into.
void loadAllTfStaticsIntoBuffer(
  const std::string & rosbag_path,
  tf2_ros::Buffer * tf_buffer_ptr);

/// Looks up a transform at a time in the tf_buffer.
/// @param target_frame Target frame.
/// @param source_frame Source frame.
/// @param stamp The time to evaluate the transform at.
/// @param tf_buffer The buffer in which to perform the lookup.
/// @param[out] T_target_source_ptr The output transform.
/// @return True is successfully looked up.
bool getTransformAtTime(
  const std::string & target_frame,
  const std::string & source_frame,
  const builtin_interfaces::msg::Time & stamp,
  const tf2_ros::Buffer & tf_buffer,
  Transform * T_target_source_ptr);

}  // namespace ros
}  // namespace datasets
}  // namespace nvblox

#include "nvblox_ros/impl/rosbag_reading_impl.hpp"

#endif  // NVBLOX_ROS__ROSBAG_READING_HPP_
