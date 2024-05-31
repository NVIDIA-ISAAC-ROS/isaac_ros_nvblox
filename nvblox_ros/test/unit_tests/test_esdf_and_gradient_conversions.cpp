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

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <nvblox/nvblox.h>

#include <std_msgs/msg/float32_multi_array.hpp>

#include "nvblox_ros/conversions/esdf_and_gradients_conversions.hpp"

namespace nvblox
{

float getTestValue(const Index3D & idx)
{
  constexpr int kMaxValue = 1000;
  return static_cast<float>(Index3DHash()(idx) % kMaxValue);
}

TEST(EsdfAndGradientsConversionsTest, FloatGrid) {
  // Set the grid to test values
  constexpr int kGridSize = 2;
  Unified3DGrid<float> grid(MemoryType::kUnified);
  const Index3D aabb_min_index(0, 0, 0);
  const Index3D aabb_size(kGridSize, kGridSize, kGridSize);
  grid.setAABB(aabb_min_index, aabb_size);
  for (int z = 0; z < kGridSize; z++) {
    for (int y = 0; y < kGridSize; y++) {
      for (int x = 0; x < kGridSize; x++) {
        const Index3D idx = Index3D(x, y, z);
        grid(idx) = getTestValue(idx);
      }
    }
  }

  // Setup the message
  std_msgs::msg::Float32MultiArray array_msg;
  array_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  array_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  array_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  array_msg.layout.dim[0].label = "x";
  array_msg.layout.dim[0].size = kGridSize;
  array_msg.layout.dim[0].stride = kGridSize * kGridSize * kGridSize;
  array_msg.layout.dim[1].label = "y";
  array_msg.layout.dim[1].size = kGridSize;
  array_msg.layout.dim[1].stride = kGridSize * kGridSize;
  array_msg.layout.dim[2].label = "z";
  array_msg.layout.dim[2].size = kGridSize;
  array_msg.layout.dim[2].stride = kGridSize;
  array_msg.data = grid.data().toVector();

  // Compare the message and original grid values;
  auto toMsgLinearIdx = [&array_msg](const Index3D & idx) -> int {
      return idx.x() * array_msg.layout.dim[1].stride + // NOLINT
             idx.y() * array_msg.layout.dim[2].stride + // NOLINT
             idx.z();                                  // NOLINT
    };
  for (int z = 0; z < kGridSize; z++) {
    for (int y = 0; y < kGridSize; y++) {
      for (int x = 0; x < kGridSize; x++) {
        const Index3D idx = Index3D(x, y, z);
        constexpr float kEps = 1e-6;
        EXPECT_NEAR(array_msg.data[toMsgLinearIdx(idx)], grid(idx), kEps);
      }
    }
  }
}

void setLayerToTestValues(EsdfLayer * layer_ptr)
{
  callFunctionOnAllVoxels<EsdfVoxel>(
    layer_ptr, [](const Index3D &, const Index3D & voxel_index, EsdfVoxel * voxel) {
      voxel->squared_distance_vox = getTestValue(voxel_index);
    });
}

float testValueToSignedDistance(const float test_value, const float voxel_size_m)
{
  return voxel_size_m * sqrt(test_value);
}

float getValueFromMessage(const Index3D & idx, const std_msgs::msg::Float32MultiArray & msg)
{
  CHECK_EQ(msg.layout.dim[0].label, "x");
  CHECK_EQ(msg.layout.dim[1].label, "y");
  CHECK_EQ(msg.layout.dim[2].label, "z");
  const int stride_y = msg.layout.dim[1].stride;
  const int stride_z = msg.layout.dim[2].stride;
  const int linear_idx = idx.z() + idx.y() * stride_z + idx.x() * stride_y;
  return msg.data[linear_idx];
}

TEST(EsdfAndGradientsConversionsTest, EsdfValues) {
  // Test layer with a single block.
  constexpr float kVoxelSize = 0.05;
  EsdfLayer esdf_layer(kVoxelSize, MemoryType::kUnified);
  auto block_ptr = esdf_layer.allocateBlockAtIndex(Index3D(0, 0, 0));
  setLayerToTestValues(&esdf_layer);

  // AABB
  const auto aabb = getAABBOfAllocatedBlocks(esdf_layer);

  // Convert to a ROS message
  conversions::EsdfAndGradientsConverter esdf_and_gradients_converter;
  CudaStreamOwning cuda_stream;
  constexpr float default_value = -1000;
  std_msgs::msg::Float32MultiArray array_msg =
    esdf_and_gradients_converter.esdfInAabbToMultiArrayMsg(
    esdf_layer, aabb, default_value,
    cuda_stream);
  cuda_stream.synchronize();
  cudaPeekAtLastError();

  auto is_inside_block = [](const Index3D & idx) -> bool {
      constexpr int kVoxelsPerSide = VoxelBlock<bool>::kVoxelsPerSide;
      return (idx.x() < kVoxelsPerSide) && (idx.y() < kVoxelsPerSide) && (idx.z() < kVoxelsPerSide);
    };

  // Check that the message contains the test values
  const Index3D aabb_min_vox = (aabb.min() / esdf_layer.voxel_size()).cast<int>();
  const Index3D aabb_max_vox = (aabb.max() / esdf_layer.voxel_size()).cast<int>();
  for (int x = aabb_min_vox.x(); x <= aabb_max_vox.x(); x++) {
    for (int y = aabb_min_vox.y(); y <= aabb_max_vox.y(); y++) {
      for (int z = aabb_min_vox.z(); z <= aabb_max_vox.z(); z++) {
        Index3D global_voxel_idx(x, y, z);
        const float msg_value = getValueFromMessage(global_voxel_idx, array_msg);
        if (is_inside_block(global_voxel_idx)) {
          const float test_value = getTestValue(global_voxel_idx);
          const float signed_distance_test_value =
            testValueToSignedDistance(test_value, esdf_layer.voxel_size());
          constexpr float kEps = 1e-6;
          EXPECT_NEAR(msg_value, signed_distance_test_value, kEps);
        } else {
          constexpr float kEps = 1e-6;
          EXPECT_NEAR(msg_value, default_value, kEps);
        }
      }
    }
  }
}

int main(int argc, char ** argv)
{
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace nvblox
