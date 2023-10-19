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

#include "nvblox_ros/transformer.hpp"

#include <algorithm>
#include <memory>
#include <string>


namespace nvblox
{

Transformer::Transformer(rclcpp::Node * node)
: node_(node)
{
  // Get params like "use_tf_transforms".
  use_tf_transforms_ =
    node->declare_parameter<bool>("use_tf_transforms", use_tf_transforms_);
  use_topic_transforms_ = node->declare_parameter<bool>(
    "use_topic_transforms",
    use_topic_transforms_);

  // Init the transform listeners if we ARE using TF at all.
  if (use_tf_transforms_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
}

bool Transformer::lookupTransformToGlobalFrame(
  const std::string & sensor_frame,
  const rclcpp::Time & timestamp,
  Transform * transform)
{
  if (!use_tf_transforms_ && !use_topic_transforms_) {
    // ERROR HERE, literally can't do anything.
    RCLCPP_ERROR(
      node_->get_logger(),
      "Not using TF OR topic transforms, what do you want us to use?");
    return false;
  }

  // Check if we have a transform queue.
  if (!use_topic_transforms_) {
    // Then I guess we're using TF.
    // Try to look up the pose in TF.
    return lookupTransformTf(global_frame_, sensor_frame, timestamp, transform);
  } else {
    // We're using topic transforms.
    if (sensor_frame != pose_frame_) {
      // Ok but we also need a pose_frame -> sensor_frame lookup here.
      Transform T_P_S;
      if (!lookupSensorTransform(sensor_frame, &T_P_S)) {
        return false;
      }

      // Now we have the TPS reports, we need T_G_P which is global to pose
      // frame. This comes from the queue.
      Transform T_G_P;
      if (!lookupTransformQueue(timestamp, &T_G_P)) {
        return false;
      }

      *transform = T_G_P * T_P_S;
      return true;
    } else {
      return lookupTransformQueue(timestamp, transform);
    }
  }
  return false;
}

void Transformer::transformCallback(
  const geometry_msgs::msg::TransformStamped::ConstSharedPtr transform_msg)
{
  rclcpp::Time timestamp = transform_msg->header.stamp;
  transform_queue_[timestamp.nanoseconds()] =
    transformToEigen(transform_msg->transform);
}

void Transformer::poseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr transform_msg)
{
  rclcpp::Time timestamp = transform_msg->header.stamp;
  transform_queue_[timestamp.nanoseconds()] = poseToEigen(transform_msg->pose);
}

bool Transformer::lookupTransformTf(
  const std::string & from_frame,
  const std::string & to_frame,
  const rclcpp::Time & timestamp,
  Transform * transform)
{
  geometry_msgs::msg::TransformStamped T_L_C_msg;
  try {
    std::string error_string;
    if (tf_buffer_->canTransform(
        from_frame, to_frame, timestamp,
        rclcpp::Duration::from_nanoseconds(0), &error_string))
    {
      T_L_C_msg = tf_buffer_->lookupTransform(from_frame, to_frame, timestamp);
    } else {
      RCLCPP_DEBUG_STREAM(
        node_->get_logger(),
        "Cant transform: from:" << from_frame << " to " << to_frame << ". Error string: " <<
          error_string);
      return false;
    }
  } catch (tf2::TransformException & e) {
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(),
      "Cant transform: from:" << from_frame << " to " << to_frame << ". Error: " << e.what());
    return false;
  }

  *transform = transformToEigen(T_L_C_msg.transform);
  return true;
}

bool Transformer::lookupTransformQueue(
  const rclcpp::Time & timestamp,
  Transform * transform)
{
  // Get latest transform
  if (timestamp == rclcpp::Time(0)) {
    if (transform_queue_.empty()) {
      return false;
    }

    *transform = transform_queue_.rbegin()->second;
    return true;
  } else {
    // Get closest transform
    uint64_t timestamp_ns = timestamp.nanoseconds();

    auto closest_match = transform_queue_.lower_bound(timestamp_ns);
    if (closest_match == transform_queue_.end()) {
      return false;
    }

    // If we're too far off on the timestamp:
    uint64_t distance = std::max(closest_match->first, timestamp_ns) -
      std::min(closest_match->first, timestamp_ns);
    if (distance > timestamp_tolerance_ns_) {
      return false;
    }

    // We just do nearest neighbor here.
    // TODO(holeynikova): add interpolation!
    *transform = closest_match->second;
    return true;
  }
}

bool Transformer::lookupSensorTransform(
  const std::string & sensor_frame,
  Transform * transform)
{
  auto it = sensor_transforms_.find(sensor_frame);
  if (it == sensor_transforms_.end()) {
    // Couldn't find sensor transform. Gotta look it up.
    if (!use_tf_transforms_) {
      // Well we're kind out out of options here.
      return false;
    }
    bool success = lookupTransformTf(
      pose_frame_, sensor_frame,
      node_->get_clock()->now(), transform);
    if (success) {
      sensor_transforms_[sensor_frame] = *transform;
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "Could not look up transform to sensor.");
    }
    return success;
  } else {
    *transform = it->second;
    return true;
  }
}

Transform Transformer::transformToEigen(
  const geometry_msgs::msg::Transform & msg) const
{
  return Transform(
    Eigen::Translation3f(
      msg.translation.x, msg.translation.y,
      msg.translation.z) *
    Eigen::Quaternionf(
      msg.rotation.w, msg.rotation.x,
      msg.rotation.y, msg.rotation.z));
}

Transform Transformer::poseToEigen(const geometry_msgs::msg::Pose & msg) const
{
  return Transform(
    Eigen::Translation3d(msg.position.x, msg.position.y, msg.position.z) *
    Eigen::Quaterniond(
      msg.orientation.w, msg.orientation.x,
      msg.orientation.y, msg.orientation.z));
}

}  // namespace nvblox
