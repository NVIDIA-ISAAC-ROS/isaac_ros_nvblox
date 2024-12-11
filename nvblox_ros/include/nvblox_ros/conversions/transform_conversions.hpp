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

#ifndef NVBLOX_ROS__CONVERSIONS__TRANSFORM_CONVERSIONS_HPP_
#define NVBLOX_ROS__CONVERSIONS__TRANSFORM_CONVERSIONS_HPP_


#include <nvblox/nvblox.h>

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace nvblox
{
namespace conversions
{

/// @brief Convert a transform to a ROS TransformStamped message.
/// @param T_parent_child The transform to be converted.
/// @param parent_frame The parent frame of the transform.
/// @param child_frame The child frame of the transform.
/// @param timestamp The timestamp of the transform.
/// @param transform_stamped_msg The resulting TransformStamped message.
void transformToTransformStampedMsg(
  const Transform & T_parent_child, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & timestamp,
  geometry_msgs::msg::TransformStamped * transform_stamped_msg);

}  // namespace conversions
}  // namespace nvblox

#endif  // NVBLOX_ROS__CONVERSIONS__TRANSFORM_CONVERSIONS_HPP_
