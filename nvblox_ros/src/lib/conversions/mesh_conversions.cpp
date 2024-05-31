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

#include "nvblox_ros/conversions/mesh_conversions.hpp"

namespace nvblox
{
namespace conversions
{

geometry_msgs::msg::Point32 point32MessageFromVector(const Eigen::Vector3f & vector)
{
  geometry_msgs::msg::Point32 point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

geometry_msgs::msg::Point pointMessageFromVector(const Eigen::Vector3f & vector)
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

void meshMessageFromSerializedMesh(
  const std::shared_ptr<const SerializedMesh> serialized_mesh,
  const rclcpp::Time & timestamp, const std::string & frame_name,
  const float mesh_layer_block_size, const bool resend_full_mesh,
  nvblox_msgs::msg::Mesh * mesh_msg)
{
  const size_t num_blocks = serialized_mesh->block_indices.size();

  // Clear the previous map in case we are resending the full map.
  mesh_msg->clear = resend_full_mesh;
  mesh_msg->header.stamp = timestamp;
  mesh_msg->header.frame_id = frame_name;
  mesh_msg->block_size = mesh_layer_block_size;
  mesh_msg->block_indices.resize(num_blocks);
  mesh_msg->blocks.resize(num_blocks);

  // Create one marker per mesh block
  for (size_t i_block = 0; i_block < num_blocks; ++i_block) {
    const int num_vertices = serialized_mesh->getNumVerticesInBlock(i_block);
    const int num_triangle_indices = serialized_mesh->getNumTriangleIndicesInBlock(i_block);

    mesh_msg->block_indices[i_block] =
      index3DMessageFromIndex3D(serialized_mesh->block_indices[i_block]);

    mesh_msg->blocks[i_block].vertices.resize(num_vertices);
    mesh_msg->blocks[i_block].colors.resize(num_vertices);
    mesh_msg->blocks[i_block].triangles.resize(num_triangle_indices);

    CHECK(serialized_mesh->colors.size() == serialized_mesh->vertices.size());

    for (size_t i_vert = 0; i_vert < static_cast<size_t>(num_vertices); ++i_vert) {
      mesh_msg->blocks[i_block].vertices[i_vert] =
        point32MessageFromVector(serialized_mesh->getVertex(i_block, i_vert));
      mesh_msg->blocks[i_block].colors[i_vert] =
        colorMessageFromColor(serialized_mesh->getColor(i_block, i_vert));
    }

    for (size_t i_tri = 0; i_tri < static_cast<size_t>(num_triangle_indices); ++i_tri) {
      mesh_msg->blocks[i_block].triangles[i_tri] =
        serialized_mesh->getTriangleIndex(i_block, i_tri);
    }
  }
}

void meshMessageFromBlocksToDelete(
  const std::vector<Index3D> & block_indices_to_delete,
  const rclcpp::Time & timestamp, const std::string & frame_name,
  const float mesh_layer_block_size,
  nvblox_msgs::msg::Mesh * mesh_msg)
{
  const size_t num_blocks = block_indices_to_delete.size();

  mesh_msg->header.stamp = timestamp;
  mesh_msg->header.frame_id = frame_name;
  mesh_msg->block_size = mesh_layer_block_size;
  mesh_msg->block_indices.resize(num_blocks);
  mesh_msg->blocks.resize(num_blocks);

  // Add empty blocks for each index that should be deleted
  for (size_t i_block = 0; i_block < num_blocks; ++i_block) {
    const Index3D & block_index = block_indices_to_delete[i_block];
    mesh_msg->block_indices[i_block] = index3DMessageFromIndex3D(block_index);
    mesh_msg->blocks[i_block] = nvblox_msgs::msg::MeshBlock();
  }
}

// Convert a mesh to a marker array.
void markerMessageFromSerializedMesh(
  const std::shared_ptr<const nvblox::SerializedMesh> & serialized_mesh,
  const std::string & frame_id, visualization_msgs::msg::MarkerArray * marker_msg)
{
  const size_t num_blocks = serialized_mesh->block_indices.size();
  marker_msg->markers.reserve(num_blocks);
  // Create one marker per mesh block
  for (size_t i_block = 0; i_block < num_blocks; ++i_block) {
    const int num_triangle_indices = serialized_mesh->getNumTriangleIndicesInBlock(i_block);
    if (num_triangle_indices == 0) {
      continue;
    }
    marker_msg->markers.emplace_back(visualization_msgs::msg::Marker());
    visualization_msgs::msg::Marker & marker = marker_msg->markers.back();

    marker.points.reserve(num_triangle_indices);
    marker.colors.reserve(num_triangle_indices);

    // Get vertices and colors for all triangle indices in this block.
    // Assumes UNWELDED mesh: all vertices in order.
    for (auto itr = serialized_mesh->triangleBlockItr(i_block);
      itr != serialized_mesh->triangleBlockItr(i_block + 1); ++itr)
    {
      marker.points.emplace_back(pointMessageFromVector(serialized_mesh->getVertex(i_block, *itr)));
      marker.colors.emplace_back(colorMessageFromColor(serialized_mesh->getColor(i_block, *itr)));
    }

    marker.header.frame_id = frame_id;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;

    // Create an unique identifier for the marker
    std::stringstream ns_stream;
    ns_stream << serialized_mesh->block_indices[i_block].x() << "_"
              << serialized_mesh->block_indices[i_block].y() << "_"
              << serialized_mesh->block_indices[i_block].z();
    marker.ns = ns_stream.str();
  }
}

}  // namespace conversions
}  // namespace nvblox
