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

#include "nvblox_message_adapters/nvblox_voxel_layer_adapter_node.hpp"

#include <isaac_ros_common/qos.hpp>

namespace nvidia::nvblox {

NvbloxVoxelAdapterNode::NvbloxVoxelAdapterNode(
    const rclcpp::NodeOptions & options, const std::string & node_name)
    : rclcpp::Node(node_name, options){
  const rclcpp::QoS input_qos =
    isaac_ros::common::AddQosParameter(*this, "SYSTEM_DEFAULT", "input_qos");
  subscriber_ = create_subscription<nvblox_msgs::msg::VoxelBlockLayer>(
    "voxel_block_layer", input_qos,
    std::bind(&NvbloxVoxelAdapterNode::messageCallback, this, std::placeholders::_1));
  publisher_ = create_publisher<nvblox_msgs::msg::VoxelSerialized>("~/voxel_serialized", 10);
  num_voxels_ = 0;
}

void NvbloxVoxelAdapterNode::messageCallback(
    const nvblox_msgs::msg::VoxelBlockLayer::ConstSharedPtr& msg) {
  // Clear all the blocks if requested
  if (msg->clear) {
    blocks_.clear();
    num_voxels_ = 0;
  }

  // Go through the list of blocks of the VoxelBlockLayer message, replace the one already existing
  // and add the new ones.
  for (size_t id = 0; id < msg->block_indices.size(); id++) {
    const auto& idx = msg->block_indices[id];
    auto& block = blocks_[{idx.x, idx.y, idx.z}];
    num_voxels_ -= block.num_voxels;

    const auto& voxel_block = msg->blocks[id];
    block.num_voxels = voxel_block.centers.size();
    num_voxels_ += block.num_voxels;
    // Serialize the message into a single float array for both the voxel positions and colors.
    block.points.resize(3 * block.num_voxels);
    block.colors.resize(4 * block.num_voxels);
    for (int bid = 0; bid < block.num_voxels; bid++) {
      block.points[3 * bid + 0] = voxel_block.centers[bid].x;
      block.points[3 * bid + 1] = voxel_block.centers[bid].y;
      block.points[3 * bid + 2] = voxel_block.centers[bid].z;
      block.colors[4 * bid + 0] = voxel_block.colors[bid].r;
      block.colors[4 * bid + 1] = voxel_block.colors[bid].g;
      block.colors[4 * bid + 2] = voxel_block.colors[bid].b;
      block.colors[4 * bid + 3] = voxel_block.colors[bid].a;
    }
    // If the block is now empty, we can remove it.
    if (block.num_voxels == 0) {
      blocks_.erase({idx.x, idx.y, idx.z});
    }
  }

  // Create the VoxelSerialized message
  nvblox_msgs::msg::VoxelSerialized msg_serialized;
  msg_serialized.header = msg->header;
  msg_serialized.num_voxels = num_voxels_;
  msg_serialized.voxel_size_m = msg->voxel_size_m;
  msg_serialized.points.reserve(3 * num_voxels_);
  msg_serialized.colors.reserve(4 * num_voxels_);
  for (const auto& kv : blocks_) {
    msg_serialized.points.insert(
        msg_serialized.points.end(), kv.second.points.begin(), kv.second.points.end());
    msg_serialized.colors.insert(
        msg_serialized.colors.end(), kv.second.colors.begin(), kv.second.colors.end());
  }
  publisher_->publish(msg_serialized);
}

}  // namespace nvidia::nvblox