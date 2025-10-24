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

#ifndef NVBLOX_MESSAGE_ADAPTERS__NVBLOX_VOXEL_ADAPTER_NODE_HPP_
#define NVBLOX_MESSAGE_ADAPTERS__NVBLOX_VOXEL_ADAPTER_NODE_HPP_

#include <map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nvblox_msgs/msg/voxel_serialized.hpp>
#include <nvblox_msgs/msg/voxel_block_layer.hpp>

namespace nvidia::nvblox {

// This node is used to convert VoxelBlockLayer messages into a VoxelSerialized message.
// When a new message comes in, it updates the list of voxels and outputs a new VoxelSerialized
// message.
class NvbloxVoxelAdapterNode : public rclcpp::Node {
public:
  explicit NvbloxVoxelAdapterNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "nvblox_voxel_layer_adapter");
  virtual ~NvbloxVoxelAdapterNode() = default;

protected:
  // Helper structure to store the information about a specific block of the VoxelBlockLayer.
  struct Block {
    std::vector<float> points;
    std::vector<float> colors;
    int32_t num_voxels;
  };

  // Callback for when a new message is received
  void messageCallback(const nvblox_msgs::msg::VoxelBlockLayer::ConstSharedPtr& msg);

  // Publisher/Subscriber
  rclcpp::Publisher<nvblox_msgs::msg::VoxelSerialized>::SharedPtr publisher_;
  rclcpp::Subscription<nvblox_msgs::msg::VoxelBlockLayer>::SharedPtr subscriber_;

  // Total number of voxels (used to allocate the memory)
  int32_t num_voxels_;
  // List of all the blocks currently containing voxels.
  std::map<std::tuple<int, int, int>, Block> blocks_;
};

}  // namespace nvidia::nvblox

#endif  // NVBLOX_MESSAGE_ADAPTERS__NVBLOX_VOXEL_ADAPTER_NODE_HPP_
