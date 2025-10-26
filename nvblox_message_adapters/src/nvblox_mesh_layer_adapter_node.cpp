// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "nvblox_message_adapters/nvblox_mesh_layer_adapter_node.hpp"

#include <isaac_ros_common/qos.hpp>

namespace nvidia::nvblox {

NvbloxMeshLayerAdapterNode::NvbloxMeshLayerAdapterNode(
    const rclcpp::NodeOptions & options, const std::string & node_name)
    : rclcpp::Node(node_name, options){
  const rclcpp::QoS input_qos =
    isaac_ros::common::AddQosParameter(*this, "SYSTEM_DEFAULT", "input_qos");
  subscriber_ = create_subscription<nvblox_msgs::msg::Mesh>(
    "mesh", input_qos,
    std::bind(&NvbloxMeshLayerAdapterNode::messageCallback, this, std::placeholders::_1));
  publisher_ = create_publisher<nvblox_msgs::msg::MeshSerialized>("~/mesh_serialized", 10);
  num_vertices_ = 0;
}

void NvbloxMeshLayerAdapterNode::messageCallback(
    const nvblox_msgs::msg::Mesh::ConstSharedPtr& msg) {
  // Clear all the blocks if requested
  if (msg->clear) {
    blocks_.clear();
    num_vertices_ = 0;
    num_triangles_ = 0;
  }
  // Go through the list of blocks of the Mesh message, replace the one already existing
  // and add the new ones.
  for (size_t id = 0; id < msg->block_indices.size(); id++) {
    const auto& idx = msg->block_indices[id];
    auto& block_to_modify = blocks_[{idx.x, idx.y, idx.z}];
    num_vertices_ -= block_to_modify.num_vertices;
    num_triangles_ -= block_to_modify.num_triangles;

    const auto& block_from_message = msg->blocks[id];
    block_to_modify.num_vertices = block_from_message.vertices.size();
    block_to_modify.num_triangles = block_from_message.triangles.size() / 3;
    // If the block is now empty, we can remove it.
    if (block_to_modify.num_triangles == 0) {
      blocks_.erase({idx.x, idx.y, idx.z});
      continue;
    }
    // Serialize the message into a single float array for the vertices, colors, and triangles.
    num_vertices_ += block_to_modify.num_vertices;
    num_triangles_ += block_to_modify.num_triangles;
    block_to_modify.vertices.resize(3 * block_to_modify.num_vertices);
    block_to_modify.colors.resize(4 * block_to_modify.num_vertices);
    for (int bid = 0; bid < block_to_modify.num_vertices; bid++) {
      block_to_modify.vertices[3 * bid + 0] = block_from_message.vertices[bid].x;
      block_to_modify.vertices[3 * bid + 1] = block_from_message.vertices[bid].y;
      block_to_modify.vertices[3 * bid + 2] = block_from_message.vertices[bid].z;
      block_to_modify.colors[4 * bid + 0] = block_from_message.colors[bid].r;
      block_to_modify.colors[4 * bid + 1] = block_from_message.colors[bid].g;
      block_to_modify.colors[4 * bid + 2] = block_from_message.colors[bid].b;
      block_to_modify.colors[4 * bid + 3] = block_from_message.colors[bid].a;
    }
    block_to_modify.triangles = block_from_message.triangles;
  }

  // Create the MeshSerialized message
  nvblox_msgs::msg::MeshSerialized msg_serialized;
  msg_serialized.header = msg->header;
  msg_serialized.num_vertices = num_vertices_;
  msg_serialized.num_triangles = num_triangles_;
  msg_serialized.triangles.reserve(3 * num_triangles_);
  msg_serialized.vertices.reserve(3 * num_vertices_);
  msg_serialized.colors.reserve(4 * num_vertices_);
  int vertices_id = 0;
  // Loop through the blocks and insert them in the overall mesh
  for (const auto& kv : blocks_) {
    msg_serialized.vertices.insert(
        msg_serialized.vertices.end(), kv.second.vertices.begin(), kv.second.vertices.end());
    msg_serialized.colors.insert(
        msg_serialized.colors.end(), kv.second.colors.begin(), kv.second.colors.end());
    // For triangles, we need to offset them by the number of vertices already added.
    for (int i : kv.second.triangles) {
      msg_serialized.triangles.push_back(i + vertices_id);
    }
    vertices_id += kv.second.num_vertices;
  }
  publisher_->publish(msg_serialized);
}

}  // namespace nvidia::nvblox