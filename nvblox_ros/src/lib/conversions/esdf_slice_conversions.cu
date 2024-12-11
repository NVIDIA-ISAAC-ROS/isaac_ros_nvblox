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

#include <nvblox/gpu_hash/internal/cuda/gpu_indexing.cuh>

#include "nvblox_ros/conversions/esdf_slice_conversions.hpp"

namespace nvblox {
namespace conversions {

constexpr int8_t kOccupancyGridUnknownValue = -1;

// Calling rules:
// - Should be called with 2D grid of thread-blocks where the total number of
// threads in each dimension exceeds the number of pixels in each image
// dimension ie:
// - blockIdx.x * blockDim.x + threadIdx.x > cols
// - blockIdx.y * blockDim.y + threadIdx.y > rows
// - We assume that theres enough space in the pointcloud to store a point per
// pixel if required.
__global__ void pointcloudFromSliceImageKernel(
    const float* slice_image_ptr,       // NOLINT
    const int rows,                     // NOLINT
    const int cols,                     // NOLINT
    const AxisAlignedBoundingBox aabb,  // NOLINT
    const float slice_height,           // NOLINT
    const float voxel_size,             // NOLINT
    const float unknown_value,          // NOLINT
    PclPointXYZI* pointcloud_ptr,       // NOLINT
    int* max_index) {
  // Get the pixel addressed by this thread.
  const int col_idx = blockIdx.x * blockDim.x + threadIdx.x;
  const int row_idx = blockIdx.y * blockDim.y + threadIdx.y;
  if (col_idx >= cols || row_idx >= rows) {
    return;
  }

  // Access the slice
  const float pixel_value =
      image::access(row_idx, col_idx, cols, slice_image_ptr);

  // If map slice not observed don't add this point
  constexpr float kEps = 1e-2;
  if (fabsf(pixel_value - unknown_value) < kEps) {
    return;
  }

  // 3D point associated with this pixel.
  PclPointXYZI point;
  point.x = aabb.min().x() + voxel_size * col_idx;
  point.y = aabb.min().y() + voxel_size * row_idx;
  point.z = slice_height;

  // Set the points intensity
  point.intensity = pixel_value;

  // Write the point to the ouput
  const int next_index = atomicAdd(max_index, 1);
  PclPointXYZI* point_out = &pointcloud_ptr[next_index];
  *point_out = point;
}

// Calling rules:
// - Should be called with 2D grid of thread-blocks where the total number of
// threads in each dimension exceeds the number of pixels in each image
// dimension ie:
// - blockIdx.x * blockDim.x + threadIdx.x > cols
// - blockIdx.y * blockDim.y + threadIdx.y > rows
// - We assume that theres enough space in the pointcloud to store a point per
// pixel if required.
__global__ void occupancyGridFromSliceImageKernel(
    const float* slice_image_ptr,         // NOLINT
    const int rows,                       // NOLINT
    const int cols,                       // NOLINT
    const float unknown_value,            // NOLINT
    int8_t* occupancy_grid_device_ptr) {  // NOLINT
  // Get the pixel addressed by this thread.
  const int col_idx = blockIdx.x * blockDim.x + threadIdx.x;
  const int row_idx = blockIdx.y * blockDim.y + threadIdx.y;
  if (col_idx >= cols || row_idx >= rows) {
    return;
  }

  // Access the slice
  const float pixel_value =
      image::access(row_idx, col_idx, cols, slice_image_ptr);

  constexpr float kEps = 1e-2;
  constexpr int8_t kOccupiedValue = 100;

  // If the distance is under epsilon (near zero), we're in occupied space so
  // set it to kOccupiedValue (via implicit cast from Bool(True) to int(1)).
  int8_t value = (pixel_value < kEps) * kOccupiedValue;
  // If the value is approximately equal to the constant signifying unknown,
  // set it to unknown in ros (-1).
  if (fabsf(pixel_value - unknown_value) < kEps) {
    value = kOccupancyGridUnknownValue;
  }

  // Write the point to the ouput
  // int8_t* cell_out = &occupancy_grid_device_ptr[row_idx * cols + col_idx];
  //*cell_out = value;
  image::access(row_idx, col_idx, cols, occupancy_grid_device_ptr) = value;
}

EsdfSliceConverter::EsdfSliceConverter()
    : EsdfSliceConverter(std::make_shared<CudaStreamOwning>()) {}

EsdfSliceConverter::EsdfSliceConverter(std::shared_ptr<CudaStream> cuda_stream)
    : cuda_stream_(cuda_stream), esdf_slicer_(cuda_stream) {}

void EsdfSliceConverter::sliceLayerToDistanceImage(
    const EsdfLayer& layer, float slice_height, float unknown_value,
    Image<float>* output_image, AxisAlignedBoundingBox* aabb) {
  CHECK_NOTNULL(aabb);
  *aabb = esdf_slicer_.getAabbOfLayerAtHeight(layer, slice_height);
  esdf_slicer_.sliceLayerToDistanceImage(layer, slice_height, unknown_value,
                                         *aabb, output_image);
}

void EsdfSliceConverter::sliceLayersToCombinedDistanceImage(
    const EsdfLayer& layer_1, const EsdfLayer& layer_2,
    float layer_1_slice_height, float layer_2_slice_height, float unknown_value,
    Image<float>* output_image, AxisAlignedBoundingBox* aabb) {
  CHECK_NOTNULL(aabb);
  *aabb = esdf_slicer_.getCombinedAabbOfLayersAtHeight(
      layer_1, layer_2, layer_1_slice_height, layer_2_slice_height);
  esdf_slicer_.sliceLayersToCombinedDistanceImage(
      layer_1, layer_2, layer_1_slice_height, layer_2_slice_height,
      unknown_value, *aabb, output_image);
}

void EsdfSliceConverter::distanceMapSliceMsgFromSliceImage(
    const Image<float>& slice_image, const AxisAlignedBoundingBox& aabb,
    float slice_height, float voxel_size, float unknown_value,
    nvblox_msgs::msg::DistanceMapSlice* map_slice) {
  CHECK_NOTNULL(map_slice);

  // Set up the message
  const int width = slice_image.cols();
  const int height = slice_image.rows();

  map_slice->origin.x = aabb.min().x();
  map_slice->origin.y = aabb.min().y();
  map_slice->origin.z = slice_height;

  map_slice->resolution = voxel_size;
  map_slice->width = width;
  map_slice->height = height;
  map_slice->unknown_value = unknown_value;

  // Allocate the map directly, we will write directly to this output to prevent
  // copies.
  map_slice->data.resize(width * height, unknown_value);

  // Copy into the message
  checkCudaErrors(cudaMemcpyAsync(
      map_slice->data.data(), slice_image.dataConstPtr(),
      slice_image.numel() * sizeof(float), cudaMemcpyDefault, *cuda_stream_));
  cuda_stream_->synchronize();
}

void EsdfSliceConverter::pointcloudMsgFromSliceImage(
    const Image<float>& slice_image, const AxisAlignedBoundingBox& aabb,
    float slice_height, float voxel_size, float unknown_value,
    sensor_msgs::msg::PointCloud2* pointcloud_msg) {
  CHECK_NOTNULL(pointcloud_msg);

  // Can happen that this function is called before the ESDF contains data.
  // Return without doing anything if that happens.
  if (slice_image.numel() <= 0 || slice_image.rows() <= 0 ||
      slice_image.cols() <= 0) {
    return;
  }

  // Allocate max space we could take up
  pcl_pointcloud_device_.reserveAsync(slice_image.numel(), *cuda_stream_);

  // Allocate output space for the number of valid points
  if (!max_index_device_) {
    max_index_device_ = make_unified<int>(MemoryType::kDevice);
  }
  max_index_device_.setZero();

  // Kernel
  // Call params
  // - 1 thread per pixel
  // - 8 x 8 threads per thread block
  // - N x M thread blocks get 1 thread per pixel
  constexpr dim3 kThreadsPerThreadBlock(8, 8, 1);
  const dim3 num_blocks(
      slice_image.cols() / kThreadsPerThreadBlock.x + 1,  // NOLINT
      slice_image.rows() / kThreadsPerThreadBlock.y + 1,  // NOLINT
      1);
  pointcloudFromSliceImageKernel<<<num_blocks, kThreadsPerThreadBlock, 0,
                                   *cuda_stream_>>>(
      slice_image.dataConstPtr(),     // NOLINT
      slice_image.rows(),             // NOLINT
      slice_image.cols(),             // NOLINT
      aabb,                           // NOLINT
      slice_height,                   // NOLINT
      voxel_size,                     // NOLINT
      unknown_value,                  // NOLINT
      pcl_pointcloud_device_.data(),  // NOLINT
      max_index_device_.get()         // NOLINT
  );
  cuda_stream_->synchronize();
  checkCudaErrors(cudaPeekAtLastError());

  // Retrieve how many points were actually used
  max_index_host_ = max_index_device_.clone(MemoryType::kHost);

  // Resize the pointcloud
  pcl_pointcloud_device_.resizeAsync(*max_index_host_, *cuda_stream_);

  // Put the points in the message
  copyDevicePointcloudToMsg(pcl_pointcloud_device_, pointcloud_msg);
}

void EsdfSliceConverter::occupancyGridFromSliceImage(
    const Image<float>& slice_image, signed char* occupancy_grid_data,
    const float unknown_value) {
  CHECK_NOTNULL(occupancy_grid_data);

  // Can happen that this function is called before the ESDF contains data.
  // Return without doing anything if that happens.
  if (slice_image.numel() <= 0 || slice_image.rows() <= 0 ||
      slice_image.cols() <= 0) {
    return;
  }

  const int width = slice_image.cols();
  const int height = slice_image.rows();

  // Allocate device-side scratch pad
  occupancy_grid_device_.reserveAsync(width * height, *cuda_stream_);

  // Call CUDA kernel to convert from float distance to int8 occupancy.
  // Kernel
  // Call params
  // - 1 thread per pixel
  // - 8 x 8 threads per thread block
  // - N x M thread blocks get 1 thread per pixel
  constexpr dim3 kThreadsPerThreadBlock(8, 8, 1);
  const dim3 num_blocks(
      slice_image.cols() / kThreadsPerThreadBlock.x + 1,  // NOLINT
      slice_image.rows() / kThreadsPerThreadBlock.y + 1,  // NOLINT
      1);
  occupancyGridFromSliceImageKernel<<<num_blocks, kThreadsPerThreadBlock, 0,
                                      *cuda_stream_>>>(
      slice_image.dataConstPtr(),    // NOLINT
      slice_image.rows(),            // NOLINT
      slice_image.cols(),            // NOLINT
      unknown_value,                 // NOLINT
      occupancy_grid_device_.data()  // NOLINT
  );
  checkCudaErrors(cudaPeekAtLastError());

  // Copy into the message
  checkCudaErrors(cudaMemcpyAsync(
      occupancy_grid_data, occupancy_grid_device_.data(),
      width * height * sizeof(int8_t), cudaMemcpyDefault, *cuda_stream_));
  cuda_stream_->synchronize();
}

}  // namespace conversions
}  // namespace nvblox
