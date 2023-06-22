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

#ifndef NVBLOX_ROS__CONVERSIONS__ESDF_SLICE_CONVERSIONS_HPP_
#define NVBLOX_ROS__CONVERSIONS__ESDF_SLICE_CONVERSIONS_HPP_

#include <nvblox/nvblox.h>

#include <nvblox_msgs/DistanceMapSlice.h>

#include "nvblox_ros/conversions/pointcloud_conversions.hpp"

namespace nvblox
{
namespace conversions
{

constexpr float kDistanceMapSliceUnknownValue = 1000.0f;

// Helper class to store all the buffers.
class EsdfSliceConverter
{
public:
  EsdfSliceConverter();

  // Create a distance map slice from an ESDF layer. Only works with z-axis
  // slices for now.
  void distanceMapSliceFromLayer(
    const EsdfLayer & layer, float z_slice_level,
    nvblox_msgs::DistanceMapSlice * map_slice);

  void distanceMapSliceFromLayers(
    const EsdfLayer & layer_1,
    const EsdfLayer & layer_2, float z_slice_level,
    Image<float> * map_slice_image_ptr,
    AxisAlignedBoundingBox * aabb_ptr);

  void distanceMapSliceImageFromLayer(
    const EsdfLayer & layer,
    float z_slice_level,
    const AxisAlignedBoundingBox & aabb,
    Image<float> * map_slice_image_ptr);

  void distanceMapSliceImageFromLayer(
    const EsdfLayer & layer,
    float z_slice_level,
    Image<float> * map_slice_image_ptr,
    AxisAlignedBoundingBox * aabb_ptr);

  void distanceMapSliceImageToMsg(
    const Image<float> & map_slice_image, const AxisAlignedBoundingBox & aabb,
    float z_slice_level, float voxel_size,
    nvblox_msgs::DistanceMapSlice * map_slice);

  void sliceImageToPointcloud(
    const Image<float> & map_slice_image,
    const AxisAlignedBoundingBox & aabb,
    float z_slice_level, float voxel_size,
    sensor_msgs::PointCloud2 * pointcloud_msg);

  AxisAlignedBoundingBox getBoundingBoxOfLayerAtHeight(
    const EsdfLayer & layer, const float z_slice_level);

private:
  // Output methods to access GPU layer *slice* in a more efficient way.
  // The output is a float image whose size *should* match the AABB with
  // a given resolution (in meters). Otherwise behavior is undefined.
  void populateSliceFromLayer(
    const EsdfLayer & layer,
    const AxisAlignedBoundingBox & aabb,
    float z_slice_height, float resolution,
    float unobserved_value, Image<float> * image);

  cudaStream_t cuda_stream_ = nullptr;

  // Buffers
  unified_ptr<int> max_index_device_;
  unified_ptr<int> max_index_host_;
  device_vector<PclPointXYZI> pcl_pointcloud_device_;
};

}  // namespace conversions
}  // namespace nvblox

#endif  // NVBLOX_ROS__CONVERSIONS__ESDF_SLICE_CONVERSIONS_HPP_
