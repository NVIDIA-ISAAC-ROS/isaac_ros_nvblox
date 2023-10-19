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

#ifndef NVBLOX_ROS__CONVERSIONS__LAYER_CONVERSIONS_HPP_
#define NVBLOX_ROS__CONVERSIONS__LAYER_CONVERSIONS_HPP_

#include <nvblox/nvblox.h>

#include <memory>

#include "nvblox_ros/conversions/pointcloud_conversions.hpp"

namespace nvblox
{
namespace conversions
{

// Helper class to store all the buffers.
class LayerConverter
{
public:
  LayerConverter();
  explicit LayerConverter(std::shared_ptr<CudaStream> cuda_stream);

  // Convert a layer to a pointcloud.
  template<typename VoxelType>
  void pointcloudMsgFromLayer(
    const VoxelBlockLayer<VoxelType> & layer,
    sensor_msgs::msg::PointCloud2 * pointcloud_msg);

  // Convert a layer in AABB to a pointcloud.
  template<typename VoxelType>
  void pointcloudMsgFromLayerInAABB(
    const VoxelBlockLayer<VoxelType> & layer,
    const AxisAlignedBoundingBox & aabb,
    sensor_msgs::msg::PointCloud2 * pointcloud_msg);

private:
  std::shared_ptr<CudaStream> cuda_stream_;

  // Buffers
  device_vector<PclPointXYZI> pcl_pointcloud_device_;
  unified_ptr<int> max_index_device_;
  unified_ptr<int> max_index_host_;
  device_vector<Index3D> block_indices_device_;
};

}  // namespace conversions
}  // namespace nvblox

#include "nvblox_ros/conversions/impl/layer_conversions_impl.hpp"

#endif  // NVBLOX_ROS__CONVERSIONS__LAYER_CONVERSIONS_HPP_
