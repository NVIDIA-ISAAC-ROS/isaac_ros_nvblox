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

#include "nvblox_ros/conversions/transform_conversions.hpp"

namespace nvblox
{
namespace conversions
{

void transformToTransformStampedMsg(
  const Transform & T_parent_child,
  const std::string & parent_frame, const std::string & child_frame,
  const rclcpp::Time & timestamp,
  geometry_msgs::msg::TransformStamped * transform_stamped_msg)
{
  transform_stamped_msg->header.stamp = timestamp;
  transform_stamped_msg->header.frame_id = parent_frame;
  transform_stamped_msg->child_frame_id = child_frame;
  transform_stamped_msg->transform.translation.x = T_parent_child.translation().x();
  transform_stamped_msg->transform.translation.y = T_parent_child.translation().y();
  transform_stamped_msg->transform.translation.z = T_parent_child.translation().z();
  Eigen::Quaternionf q(T_parent_child.rotation());
  transform_stamped_msg->transform.rotation.x = q.x();
  transform_stamped_msg->transform.rotation.y = q.y();
  transform_stamped_msg->transform.rotation.z = q.z();
  transform_stamped_msg->transform.rotation.w = q.w();
}

}  // namespace conversions
}  // namespace nvblox
