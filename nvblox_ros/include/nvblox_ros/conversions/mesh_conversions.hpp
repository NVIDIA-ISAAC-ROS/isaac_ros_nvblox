// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVBLOX_ROS__CONVERSIONS__MESH_CONVERSIONS_HPP_
#define NVBLOX_ROS__CONVERSIONS__MESH_CONVERSIONS_HPP_

#include <nvblox/nvblox.h>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nvblox_msgs/msg/mesh.hpp>


namespace nvblox
{
namespace conversions
{

// Convert a serialized mesh to a mesh message
void meshMessageFromSerializedMesh(
  const std::shared_ptr<const SerializedMesh> serialized_mesh,
  const rclcpp::Time & timestamp,
  const std::string & frame_name,
  const float mesh_layer_block_size,
  const bool resend_full_mesh,
  nvblox_msgs::msg::Mesh * mesh_msg);

// Convert a block_indices_to_delete vector to a mesh message (deleting from the visualization)
void meshMessageFromBlocksToDelete(
  const std::vector<Index3D> & block_indices_to_delete,
  const rclcpp::Time & timestamp,
  const std::string & frame_name,
  const float mesh_layer_block_size,
  nvblox_msgs::msg::Mesh * mesh_msg);

// Convert a mesh to a marker array.
void markerMessageFromSerializedMesh(
  const std::shared_ptr<const nvblox::SerializedMesh> & serialized_mesh,
  const std::string & frame_id,
  visualization_msgs::msg::MarkerArray * marker_msg);

}  // namespace conversions
}  // namespace nvblox

#endif  // NVBLOX_ROS__CONVERSIONS__MESH_CONVERSIONS_HPP_
