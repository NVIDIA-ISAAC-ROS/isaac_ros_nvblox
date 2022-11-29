// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef NVBLOX_ROS__IMPL__CONVERSIONS_IMPL_HPP_
#define NVBLOX_ROS__IMPL__CONVERSIONS_IMPL_HPP_

#include <nvblox/core/accessors.h>

namespace nvblox
{

// Helper function to output all valid points.
template<typename VoxelType>
inline void RosConverter::pointcloudFromLayerInAABB(
  const VoxelBlockLayer<VoxelType> & layer, const AxisAlignedBoundingBox & aabb,
  sensor_msgs::msg::PointCloud2 * pointcloud_msg)
{
  CHECK_NOTNULL(pointcloud_msg);
  convertLayerInAABBToPCLCuda<VoxelType>(layer, aabb, pointcloud_msg);
}

// Convert an SDF to a pointcloud.
template<typename VoxelType>
inline void RosConverter::pointcloudFromLayer(
  const VoxelBlockLayer<VoxelType> & layer,
  sensor_msgs::msg::PointCloud2 * pointcloud_msg)
{
  AxisAlignedBoundingBox aabb;
  aabb.setEmpty();
  pointcloudFromLayerInAABB<VoxelType>(layer, aabb, pointcloud_msg);
}

geometry_msgs::msg::Point32 point32MessageFromVector(
  const Eigen::Vector3f & vector)
{
  geometry_msgs::msg::Point32 point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

geometry_msgs::msg::Point pointMessageFromVector(
  const Eigen::Vector3f & vector)
{
  geometry_msgs::msg::Point point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

std_msgs::msg::ColorRGBA colorMessageFromColor(const Color & color)
{
  std_msgs::msg::ColorRGBA color_msg;
  color_msg.r = static_cast<float>(color.r) / 255.0f;
  color_msg.g = static_cast<float>(color.g) / 255.0f;
  color_msg.b = static_cast<float>(color.b) / 255.0f;
  color_msg.a = 1.0f;
  return color_msg;
}

nvblox_msgs::msg::Index3D index3DMessageFromIndex3D(const Index3D & index)
{
  nvblox_msgs::msg::Index3D index_msg;
  index_msg.x = index.x();
  index_msg.y = index.y();
  index_msg.z = index.z();
  return index_msg;
}

}  // namespace nvblox

#endif  // NVBLOX_ROS__IMPL__CONVERSIONS_IMPL_HPP_
