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

#ifndef NVBLOX_ROS__CONVERSIONS__MESH_CONVERSIONS_HPP_
#define NVBLOX_ROS__CONVERSIONS__MESH_CONVERSIONS_HPP_

#include <nvblox/nvblox.h>

#include <string>
#include <vector>

#include <nvblox_msgs/msg/mesh.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


namespace nvblox
{
namespace conversions
{

// Convert a mesh to a message.
void meshMessageFromMeshLayer(
  const BlockLayer<MeshBlock> & mesh_layer,
  nvblox_msgs::msg::Mesh * mesh_msg);

void meshMessageFromMeshBlocks(
  const BlockLayer<MeshBlock> & mesh_layer,
  const std::vector<Index3D> & block_indices,
  nvblox_msgs::msg::Mesh * mesh_msg,
  const std::vector<Index3D> & deleted_indices = std::vector<Index3D>());

// Convert a mesh to a marker array.
void markerMessageFromMeshLayer(
  const BlockLayer<MeshBlock> & mesh_layer, const std::string & frame_id,
  visualization_msgs::msg::MarkerArray * marker_msg);

}  // namespace conversions
}  // namespace nvblox

#endif  // NVBLOX_ROS__CONVERSIONS__MESH_CONVERSIONS_HPP_
