// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVBLOX_ROS__CONVERSIONS__ESDF_SLICE_CONVERSIONS_HPP_
#define NVBLOX_ROS__CONVERSIONS__ESDF_SLICE_CONVERSIONS_HPP_

#include <nvblox/nvblox.h>

#include <memory>

#include <nvblox_msgs/msg/distance_map_slice.hpp>

#include "nvblox_ros/conversions/pointcloud_conversions.hpp"

namespace nvblox
{
namespace conversions
{

class EsdfSliceConverter
{
public:
  EsdfSliceConverter();
  explicit EsdfSliceConverter(std::shared_ptr<CudaStream> cuda_stream);

  // ------------- WRAPPING ESDF SLICER FUNCTIONS -------------

  // Slicing an esdf layer (using EsdfSlicer)
  void sliceLayerToDistanceImage(
    const EsdfLayer & layer, float slice_height,
    Image<float> * output_image,
    AxisAlignedBoundingBox * aabb);

  // Slicing multiple esdf layer (using EsdfSlicer)
  void sliceLayersToCombinedDistanceImage(
    const EsdfLayer & layer_1,
    const EsdfLayer & layer_2,
    float slice_height,
    Image<float> * output_image,
    AxisAlignedBoundingBox * aabb);

  // ------------- CONVERSIONS TO ROS MESSAGES -------------

  // Convert slice image to distance map message
  void distanceMapSliceMsgFromSliceImage(
    const Image<float> & slice_image, const AxisAlignedBoundingBox & aabb,
    float slice_height, float voxel_size,
    nvblox_msgs::msg::DistanceMapSlice * map_slice);

  // Convert slice image to pointcloud
  void pointcloudMsgFromSliceImage(
    const Image<float> & slice_image, const AxisAlignedBoundingBox & aabb,
    float slice_height, float voxel_size,
    sensor_msgs::msg::PointCloud2 * pointcloud_msg);

private:
  // Slicer that does the work
  EsdfSlicer esdf_slicer_;

  std::shared_ptr<CudaStream> cuda_stream_;

  // Buffers
  unified_ptr<int> max_index_device_;
  unified_ptr<int> max_index_host_;
  device_vector<PclPointXYZI> pcl_pointcloud_device_;
};

}  // namespace conversions
}  // namespace nvblox

#endif  // NVBLOX_ROS__CONVERSIONS__ESDF_SLICE_CONVERSIONS_HPP_
