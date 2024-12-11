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

#ifndef NVBLOX_ROS__IMPL__ROSBAG_READING_IMPL_HPP_
#define NVBLOX_ROS__IMPL__ROSBAG_READING_IMPL_HPP_

namespace nvblox
{
namespace datasets
{
namespace ros
{

template<typename MessageType>
MessageType deserializeMessage(
  const rosbag2_storage::SerializedBagMessage & serialized_msg)
{
  MessageType msg;
  const rclcpp::SerializedMessage extracted_serialized_msg(
    *serialized_msg.serialized_data);
  rclcpp::Serialization<MessageType> serialization;
  serialization.deserialize_message(&extracted_serialized_msg, &msg);
  return msg;
}

}  // namespace ros
}  // namespace datasets
}  // namespace nvblox

#endif  // NVBLOX_ROS__IMPL__ROSBAG_READING_IMPL_HPP_
