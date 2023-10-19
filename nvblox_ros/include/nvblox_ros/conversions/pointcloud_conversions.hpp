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

#ifndef NVBLOX_ROS__CONVERSIONS__POINTCLOUD_CONVERSIONS_HPP_
#define NVBLOX_ROS__CONVERSIONS__POINTCLOUD_CONVERSIONS_HPP_

#include <nvblox/nvblox.h>

#include <memory>
#include <string>
#include <vector>
#include <unordered_set>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace nvblox
{
namespace conversions
{

// Helper struct for storing PCL points.
// 16-byte alignment to match what PCL does internally:
// https://pointclouds.org/documentation/point__types_8hpp_source.html
struct alignas (16) PclPointXYZI
{
  float x;
  float y;
  float z;
  float intensity;
};

void copyDevicePointcloudToMsg(
  const device_vector<PclPointXYZI> & pcl_pointcloud_device,
  sensor_msgs::msg::PointCloud2 * pointcloud_msg);

// Helper class to store all the buffers.
class PointcloudConverter
{
public:
  PointcloudConverter();
  explicit PointcloudConverter(std::shared_ptr<CudaStream> cuda_stream);

  // Internal pointcloud representation to a ROS pointcloud
  void pointcloudMsgFromPointcloud(
    const Pointcloud & pointcloud,
    sensor_msgs::msg::PointCloud2 * pointcloud_msg);

  // Convert pointcloud to depth image.
  void depthImageFromPointcloudGPU(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud,
    const Lidar & lidar, DepthImage * depth_image_ptr);

  // This function returns true if the pointcloud passed in is consistent with
  // the LiDAR intrinsics model.
  bool checkLidarPointcloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud,
    const Lidar & lidar);

  // Write the pointcloud to file
  void writeLidarPointcloudToFile(
    const std::string filepath,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud);

  /// Generates a marker with a bunch of cubes in it. Note that the resultant
  /// marker has does not have a frame or timestamp set (this is left to
  /// caller).
  ///
  /// @param points Voxel centers
  /// @param cube_size Edge length of the cubes in Marker msg
  /// @param color What color the cubes appear
  /// @param[out] marker_ptr Output marker
  void pointsToCubesMarkerMsg(
    const std::vector<Vector3f> & points,
    const float cube_size, const Color & color,
    visualization_msgs::msg::Marker * marker_ptr);

private:
  std::unordered_set<Lidar, Lidar::Hash> checked_lidar_models_;

  std::shared_ptr<CudaStream> cuda_stream_;

  // Buffers
  host_vector<Vector3f> lidar_pointcloud_host_;
  device_vector<Vector3f> lidar_pointcloud_device_;
  device_vector<PclPointXYZI> pcl_pointcloud_device_;
};

}  // namespace conversions
}  // namespace nvblox

#endif  // NVBLOX_ROS__CONVERSIONS__POINTCLOUD_CONVERSIONS_HPP_
