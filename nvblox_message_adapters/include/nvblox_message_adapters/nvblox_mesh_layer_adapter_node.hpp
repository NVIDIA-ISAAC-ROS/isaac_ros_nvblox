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

#ifndef NVBLOX_MESSAGE_ADAPTERS__NVBLOX_MESH_LAYER_ADAPTER_HPP_
#define NVBLOX_MESSAGE_ADAPTERS__NVBLOX_MESH_LAYER_ADAPTER_HPP_

#include <map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nvblox_msgs/msg/mesh_serialized.hpp>
#include <nvblox_msgs/msg/mesh.hpp>

namespace nvidia::nvblox {

// This node is used to convert Mesh messages into a MeshSerialized message.
// When a new message comes in, it updates the list of block and outputs a new MeshSerialized
// message.
class NvbloxMeshLayerAdapterNode : public rclcpp::Node {
public:
  explicit NvbloxMeshLayerAdapterNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "nvblox_mesh_layer_adapter");
  virtual ~NvbloxMeshLayerAdapterNode() = default;

protected:
  // Helper structure to store the information about a specific block of the Mesh messages.
  struct Block {
    std::vector<float> vertices;
    std::vector<float> colors;
    std::vector<int> triangles;
    int32_t num_vertices;
    int32_t num_triangles;
  };

  // Callback for when a new message is received
  void messageCallback(const nvblox_msgs::msg::Mesh::ConstSharedPtr& msg);

  // Publisher/Subscriber
  rclcpp::Publisher<nvblox_msgs::msg::MeshSerialized>::SharedPtr publisher_;
  rclcpp::Subscription<nvblox_msgs::msg::Mesh>::SharedPtr subscriber_;

  // Total number of vertices and triangles (used to allocate the memory)
  int32_t num_vertices_;
  int32_t num_triangles_;
  // List of all the blocks currently containing a mesh.
  std::map<std::tuple<int, int, int>, Block> blocks_;
};

}  // namespace nvidia::nvblox

#endif  // NVBLOX_MESSAGE_ADAPTERS__NVBLOX_MESH_LAYER_ADAPTER_HPP_
