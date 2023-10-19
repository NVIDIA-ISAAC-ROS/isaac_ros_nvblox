// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <nvblox/core/log_odds.h>
#include <nvblox/gpu_hash/internal/cuda/gpu_indexing.cuh>

#include "nvblox_ros/conversions/layer_conversions.hpp"

namespace nvblox {
namespace conversions {

LayerConverter::LayerConverter()
    : LayerConverter(std::make_shared<CudaStreamOwning>()) {}

LayerConverter::LayerConverter(std::shared_ptr<CudaStream> cuda_stream)
    : cuda_stream_(cuda_stream) {}

template <typename VoxelType>
__device__ bool getVoxelIntensity(const VoxelType& voxel, float voxel_size,
                                  float* intensity);

template <>
__device__ bool getVoxelIntensity(const OccupancyVoxel& voxel, float voxel_size,
                                  float* intensity) {
  constexpr float kMinProbability = 0.5f;
  *intensity = probabilityFromLogOdds(voxel.log_odds);
  return probabilityFromLogOdds(voxel.log_odds) > kMinProbability;
}

template <>
__device__ bool getVoxelIntensity(const EsdfVoxel& voxel, float voxel_size,
                                  float* intensity) {
  *intensity = voxel_size * sqrtf(voxel.squared_distance_vox);
  if (voxel.is_inside) {
    *intensity = -*intensity;
  }
  return voxel.observed;
}

template <>
__device__ bool getVoxelIntensity(const TsdfVoxel& voxel, float voxel_size,
                                  float* intensity) {
  constexpr float kMinWeight = 0.1f;
  *intensity = voxel.distance;
  return voxel.weight > kMinWeight;
}

template <>
__device__ bool getVoxelIntensity(const FreespaceVoxel& voxel, float voxel_size,
                                  float* intensity) {
  *intensity = voxel.is_high_confidence_freespace;
  // Only show non high confidence freespace
  return !voxel.is_high_confidence_freespace;
}

// Inputs: GPU hash for the E/TSDF.
//         AABB.
//         Voxel Size (just needed for ESDF).
// Outputs: vector of pcl::PointXYZIs.
//          max index (updated atomically).
template <typename VoxelType>
__global__ void copyLayerToPCLKernel(
    Index3DDeviceHashMapType<VoxelBlock<VoxelType>> block_hash,
    Index3D* block_indices, size_t num_indices, int max_output_indices,
    AxisAlignedBoundingBox aabb, float block_size, PclPointXYZI* pointcloud,
    int* max_index) {
  const float voxel_size = block_size / VoxelBlock<VoxelType>::kVoxelsPerSide;

  // Get the relevant block.
  __shared__ VoxelBlock<VoxelType>* block_ptr;
  if (threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0) {
    block_ptr = nullptr;
    auto it = block_hash.find(block_indices[blockIdx.x]);
    if (it != block_hash.end()) {
      block_ptr = it->second;
    }
  }

  __syncthreads();

  if (block_ptr == nullptr) {
    return;
  }

  // For every voxel, check if it's in the AABB.
  Index3D voxel_index(threadIdx.x, threadIdx.y, threadIdx.z);

  // Get the voxel position:
  Vector3f voxel_position = getPositionFromBlockIndexAndVoxelIndex(
      block_size, block_indices[blockIdx.x], voxel_index);

  if (!aabb.contains(voxel_position)) {
    return;
  }

  // Check if this voxel sucks or not.
  const VoxelType& voxel =
      block_ptr->voxels[voxel_index.x()][voxel_index.y()][voxel_index.z()];
  float intensity = 0.0f;
  if (!getVoxelIntensity<VoxelType>(voxel, voxel_size, 
                                    &intensity)) {
    return;
  }

  // Otherwise shove it in the output.
  int next_index = atomicAdd(max_index, 1);

  if (next_index >= max_output_indices) {
    printf("Overrunning the space. This shouldn't happen.\n");
    return;
  }
  PclPointXYZI& point = pointcloud[next_index];
  point.x = voxel_position.x();
  point.y = voxel_position.y();
  point.z = voxel_position.z();
  point.intensity = intensity;
}

template <typename VoxelType>
void LayerConverter::pointcloudMsgFromLayerInAABB(
    const VoxelBlockLayer<VoxelType>& layer, const AxisAlignedBoundingBox& aabb,
    sensor_msgs::msg::PointCloud2* pointcloud) {
  CHECK_NOTNULL(pointcloud);

  constexpr int kVoxelsPerSide = VoxelBlock<TsdfVoxel>::kVoxelsPerSide;
  constexpr int kVoxelsPerBlock =
      kVoxelsPerSide * kVoxelsPerSide * kVoxelsPerSide;
  const float voxel_size = layer.voxel_size();

  // In case the AABB is infinite, make sure we have a finite number of
  // voxels.
  AxisAlignedBoundingBox aabb_intersect = getAABBOfAllocatedBlocks(layer);
  if (!aabb.isEmpty()) {
    aabb_intersect = aabb_intersect.intersection(aabb);
  }

  // Figure out which blocks are in the AABB.
  std::vector<Index3D> block_indices =
      getAllocatedBlocksWithinAABB(layer, aabb_intersect);
  // Copy to device memory.
  block_indices_device_.copyFromAsync(block_indices, *cuda_stream_);

  if (block_indices.empty()) {
    return;
  }
  size_t num_voxels = block_indices.size() * kVoxelsPerBlock;

  // Allocate a GPU pointcloud.
  pcl_pointcloud_device_.resize(num_voxels);

  // Get the hash.
  GPULayerView<VoxelBlock<VoxelType>> gpu_layer_view = layer.getGpuLayerView();

  // Create an output size variable.
  if (!max_index_device_) {
    max_index_device_ = make_unified<int>(MemoryType::kDevice);
  }
  max_index_device_.setZero();

  // Call the kernel.
  int dim_block = block_indices.size();
  dim3 dim_threads(kVoxelsPerSide, kVoxelsPerSide, kVoxelsPerSide);

  copyLayerToPCLKernel<VoxelType><<<dim_block, dim_threads, 0, *cuda_stream_>>>(
      gpu_layer_view.getHash().impl_, block_indices_device_.data(),
      block_indices.size(), num_voxels, aabb_intersect, layer.block_size(),
      pcl_pointcloud_device_.data(), max_index_device_.get());
  checkCudaErrors(cudaStreamSynchronize(*cuda_stream_));
  checkCudaErrors(cudaPeekAtLastError());

  // Copy the pointcloud out.
  max_index_host_ = max_index_device_.clone(MemoryType::kHost);
  pcl_pointcloud_device_.resize(*max_index_host_);

  // Copy to the message
  copyDevicePointcloudToMsg(pcl_pointcloud_device_, pointcloud);
}

// Template specializations.
template void LayerConverter::pointcloudMsgFromLayerInAABB<TsdfVoxel>(
    const VoxelBlockLayer<TsdfVoxel>& layer, const AxisAlignedBoundingBox& aabb,
    sensor_msgs::msg::PointCloud2* pointcloud);

template void LayerConverter::pointcloudMsgFromLayerInAABB<FreespaceVoxel>(
    const VoxelBlockLayer<FreespaceVoxel>& layer,
    const AxisAlignedBoundingBox& aabb,
    sensor_msgs::msg::PointCloud2* pointcloud);

template void LayerConverter::pointcloudMsgFromLayerInAABB<EsdfVoxel>(
    const VoxelBlockLayer<EsdfVoxel>& layer, const AxisAlignedBoundingBox& aabb,
    sensor_msgs::msg::PointCloud2* pointcloud);

template void LayerConverter::pointcloudMsgFromLayerInAABB<OccupancyVoxel>(
    const VoxelBlockLayer<OccupancyVoxel>& layer,
    const AxisAlignedBoundingBox& aabb,
    sensor_msgs::msg::PointCloud2* pointcloud);

}  // namespace conversions
}  // namespace nvblox
