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
#include "nvblox_ros/conversions/esdf_and_gradients_conversions.hpp"

#include <nvblox/core/types.h>
#include <nvblox/map/unified_3d_grid.h>
#include <nvblox/map/voxels.h>
#include <nvblox/map/internal/cuda/impl/layer_to_3d_grid_impl.cuh>
#include <nvblox/map/internal/cuda/layer_to_3d_grid.cuh>

namespace nvblox {
namespace conversions {

struct SignedDistanceFunctor {
  SignedDistanceFunctor(float _voxel_size) : voxel_size(_voxel_size) {}
  ~SignedDistanceFunctor() = default;

  __device__ __inline__ float operator()(const EsdfVoxel& esdf_voxel) const {
    assert(esdf_voxel.squared_distance_vox >= 0.0f);
    const float unsigned_distance_vox = sqrt(esdf_voxel.squared_distance_vox);
    float distance_m = unsigned_distance_vox * voxel_size;
    if (esdf_voxel.is_inside) {
      distance_m *= -1.0f;
    }
    return distance_m;
  }

  const float voxel_size;
};

std_msgs::msg::Float32MultiArray
EsdfAndGradientsConverter::esdfInAabbToMultiArrayMsg(
    const EsdfLayer& esdf_layer,         // NOLINT
    const AxisAlignedBoundingBox& aabb,  // NOLINT
    const float default_value,           // NOLINT
    CudaStream cuda_stream) {
  // Copy the values out to the grid on the device
  SignedDistanceFunctor conversion_op(esdf_layer.voxel_size());
  voxelLayerToDenseVoxelGridInAABBAsync(esdf_layer, aabb, default_value,
                                        conversion_op, &gpu_grid_, cuda_stream);

  // GPU grid -> CPU (pinned) grid
  cpu_grid_.copyFromAsync(gpu_grid_, cuda_stream);

  // The size in voxels
  const Index3D size_in_voxels = gpu_grid_.aabb_size();

  // CPU grid -> ROS message
  std_msgs::msg::Float32MultiArray array_msg;
  array_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  array_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  array_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  array_msg.layout.dim[0].label = "x";
  array_msg.layout.dim[0].size = size_in_voxels.x();
  array_msg.layout.dim[0].stride =
      size_in_voxels.x() * size_in_voxels.y() * size_in_voxels.z();
  array_msg.layout.dim[1].label = "y";
  array_msg.layout.dim[1].size = size_in_voxels.y();
  array_msg.layout.dim[1].stride = size_in_voxels.y() * size_in_voxels.z();
  array_msg.layout.dim[2].label = "z";
  array_msg.layout.dim[2].size = size_in_voxels.z();
  array_msg.layout.dim[2].stride = size_in_voxels.z();
  array_msg.data = cpu_grid_.data().toVectorAsync(cuda_stream);

  cuda_stream.synchronize();

  return array_msg;
}

}  // namespace conversions
}  // namespace nvblox
