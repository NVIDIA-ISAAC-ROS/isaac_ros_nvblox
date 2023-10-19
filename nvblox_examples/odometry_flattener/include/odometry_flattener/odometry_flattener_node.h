// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ODOMETRY_FLATTENER__ODOMETRY_FLATTENER_NODE_HPP_
#define ODOMETRY_FLATTENER__ODOMETRY_FLATTENER_NODE_HPP_

#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace nvblox {

class OdometryFlattenerNode : public rclcpp::Node {
 public:
  OdometryFlattenerNode(const rclcpp::NodeOptions & options);

  void tfMessageCallback(tf2_msgs::msg::TFMessage::ConstSharedPtr msg);

 private:
  // Input frame names
  std::string input_parent_frame_id_ = "odom";
  std::string input_child_frame_id_ = "base_link";

  // Output frame names
  std::string output_parent_frame_id_ = "odom";
  std::string output_child_frame_id_ = "base_link_flattened";

  // Whether or not to invert the output transform. This is often necessary to
  // ensure each node in the TF tree has a single parent.
  bool invert_output_transform_ = false;

  // Subscribers
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf2_message_sub_;

  // Publishers
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace nvblox

#endif // ODOMETRY_FLATTENER__ODOMETRY_FLATTENER_NODE_HPP_