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
#include "nvblox_ros/rosbag_reading.hpp"

#include "glog/logging.h"

#include "tf2_eigen/tf2_eigen.hpp"

namespace nvblox
{
namespace datasets
{
namespace ros
{

constexpr char kTfStaticTopicName[] = "/tf_static";

void createSingleTopicReader(
  const std::string & rosbag_path, const std::string & topic_name,
  rosbag2_cpp::Reader * reader_ptr)
{
  LOG(INFO) << "Creating a ROS reader for rosbag at:" << rosbag_path
            << " , for topic: " << topic_name;
  reader_ptr->open(rosbag_path);
  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back(topic_name);
  reader_ptr->set_filter(filter);
}

uint64_t toNanoSeconds(const builtin_interfaces::msg::Time & time_msg)
{
  return static_cast<uint64_t>(time_msg.sec) * 1e9 + static_cast<uint64_t>(time_msg.nanosec);
}

bool operator<(const builtin_interfaces::msg::Time & lhs, const builtin_interfaces::msg::Time & rhs)
{
  return toNanoSeconds(lhs) < toNanoSeconds(rhs);
}

bool operator<=(
  const builtin_interfaces::msg::Time & lhs,
  const builtin_interfaces::msg::Time & rhs)
{
  return toNanoSeconds(lhs) <= toNanoSeconds(rhs);
}

builtin_interfaces::msg::Time getLowestStamp(const tf2_msgs::msg::TFMessage & msg)
{
  builtin_interfaces::msg::Time lowest_stamp = msg.transforms[0].header.stamp;
  for (const auto & transform : msg.transforms) {
    if (transform.header.stamp < lowest_stamp) {
      lowest_stamp = transform.header.stamp;
    }
  }
  return lowest_stamp;
}

builtin_interfaces::msg::Time getEarlierTime(
  const builtin_interfaces::msg::Time & msg,
  const float diff_s)
{
  const uint64_t stamp_ns = toNanoSeconds(msg);
  const uint64_t diff_ns = static_cast<uint64_t>(diff_s * 1.0e9);
  CHECK_GT(stamp_ns, diff_ns);
  const uint64_t new_stamp_ns = stamp_ns - diff_ns;
  const uint64_t sec_part = static_cast<uint64_t>(new_stamp_ns / 1e9);
  const uint64_t ns_part = new_stamp_ns - (sec_part * 1e9);
  builtin_interfaces::msg::Time new_msg;
  new_msg.sec = static_cast<int32_t>(sec_part);
  new_msg.nanosec = static_cast<int32_t>(ns_part);
  return new_msg;
}

void loadAllTfStaticsIntoBuffer(const std::string & rosbag_path, tf2_ros::Buffer * tf_buffer_ptr)
{
  DLOG(INFO) << "Starting to load all /tf_static messages";
  rosbag2_cpp::Reader tf_static_reader;
  createSingleTopicReader(rosbag_path, kTfStaticTopicName, &tf_static_reader);
  while (tf_static_reader.has_next()) {
    auto serialized_msg = tf_static_reader.read_next();
    auto tf_msg = deserializeMessage<tf2_msgs::msg::TFMessage>(*serialized_msg);
    for (const auto & transform : tf_msg.transforms) {
      constexpr bool kStaticTransform = true;
      tf_buffer_ptr->setTransform(transform, "default_authority", kStaticTransform);
    }
  }
  DLOG(INFO) << "Done loading /tf_static messages";
}

bool getTransformAtTime(
  const std::string & target_frame, const std::string & source_frame,
  const builtin_interfaces::msg::Time & stamp,
  const tf2_ros::Buffer & tf_buffer, Transform * T_target_source_ptr)
{
  CHECK_NOTNULL(T_target_source_ptr);
  const tf2::TimePoint time_point(std::chrono::nanoseconds(toNanoSeconds(stamp)));
  if (tf_buffer.canTransform(target_frame, source_frame, time_point)) {
    const geometry_msgs::msg::TransformStamped transform_stamped_msg =
      tf_buffer.lookupTransform(target_frame, source_frame, time_point);
    *T_target_source_ptr = tf2::transformToEigen(transform_stamped_msg).cast<float>();
    return true;
  }
  return false;
}

}  // namespace ros
}  // namespace datasets
}  // namespace nvblox
