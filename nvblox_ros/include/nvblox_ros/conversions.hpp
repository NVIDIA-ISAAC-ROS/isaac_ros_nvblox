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

#ifndef NVBLOX_ROS__CONVERSIONS_HPP_
#define NVBLOX_ROS__CONVERSIONS_HPP_

#include <nvblox/nvblox.h>

#include <string>
#include <vector>
#include <unordered_set>

#include <nvblox_msgs/msg/distance_map_slice.hpp>
#include <nvblox_msgs/msg/mesh.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


namespace nvblox
{

// Convert vectors.
inline geometry_msgs::msg::Point32 point32MessageFromVector(
  const Eigen::Vector3f & vector);

inline geometry_msgs::msg::Point pointMessageFromVector(
  const Eigen::Vector3f & vector);

// Convert colors.
inline std_msgs::msg::ColorRGBA colorMessageFromColor(const Color & color);

// Convert indices
inline nvblox_msgs::msg::Index3D index3DMessageFromIndex3D(
  const Index3D & index);

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

// Helper class to store all the buffers.
class RosConverter
{
public:
  RosConverter();

  // Convert camera info message to NVBlox camera object
  Camera cameraFromMessage(const sensor_msgs::msg::CameraInfo & camera_info);

  // Convert image to depth frame object
  bool depthImageFromImageMessage(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    DepthImage * depth_frame);

  bool colorImageFromImageMessage(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    ColorImage * color_image);

  // Convert depth frame to image message.
  void imageMessageFromDepthImage(
    const DepthImage & depth_frame,
    const std::string & frame_id,
    sensor_msgs::msg::Image * image_msg);

  // Convert a mesh to a message.
  void meshMessageFromMeshLayer(
    const BlockLayer<MeshBlock> & mesh_layer,
    nvblox_msgs::msg::Mesh * mesh_msg);

  void meshMessageFromMeshBlocks(
    const BlockLayer<MeshBlock> & mesh_layer,
    const std::vector<Index3D> & block_indices,
    nvblox_msgs::msg::Mesh * mesh_msg,
    const std::vector<Index3D> & deleted_indices = std::vector<Index3D>());

  void meshBlockMessageFromMeshBlock(
    const MeshBlock & mesh_block, nvblox_msgs::msg::MeshBlock * mesh_block_msg);

  // Convert a mesh to a marker array.
  void markerMessageFromMeshLayer(
    const BlockLayer<MeshBlock> & mesh_layer, const std::string & frame_id,
    visualization_msgs::msg::MarkerArray * marker_msg);

  void markerMessageFromMeshBlock(
    const MeshBlock::ConstPtr & mesh_block,
    const std::string & frame_id,
    visualization_msgs::msg::Marker * marker_msg);

  // Convert an SDF to a pointcloud.
  template<typename VoxelType>
  void pointcloudFromLayer(
    const VoxelBlockLayer<VoxelType> & layer,
    sensor_msgs::msg::PointCloud2 * pointcloud_msg);

  // Convert an SDF to a pointcloud within an AABB (use this for slices, for
  // example).
  template<typename VoxelType>
  void pointcloudFromLayerInAABB(
    const VoxelBlockLayer<VoxelType> & layer,
    const AxisAlignedBoundingBox & aabb,
    sensor_msgs::msg::PointCloud2 * pointcloud_msg);

  // Create a distance map slice from an ESDF layer. Only works with z-axis
  // slices for now.
  void distanceMapSliceFromLayer(
    const EsdfLayer & layer, float height,
    nvblox_msgs::msg::DistanceMapSlice * map_slice);

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

private:
  // Helper functions for CUDA conversions.
  template<typename VoxelType>
  void convertLayerInAABBToPCLCuda(
    const VoxelBlockLayer<VoxelType> & layer,
    const AxisAlignedBoundingBox & aabb,
    sensor_msgs::msg::PointCloud2 * pointcloud_msg);

  // Output methods to access GPU layer *slice* in a more efficient way.
  // The output is a float image whose size *should* match the AABB with
  // a given resolution (in meters). Otherwise behavior is undefined.
  void populateSliceFromLayer(
    const EsdfLayer & layer,
    const AxisAlignedBoundingBox & aabb,
    float z_slice_height, float resolution,
    float unobserved_value, Image<float> * image);

  // State.
  cudaStream_t cuda_stream_ = nullptr;

  device_vector<Index3D> block_indices_device_;
  device_vector<PclPointXYZI> layer_pointcloud_device_;
  unified_ptr<int> max_index_device_;
  unified_ptr<int> max_index_host_;

  // Pointcloud conversion
  std::unordered_set<Lidar, Lidar::Hash> checked_lidar_models_;
  host_vector<Vector3f> lidar_pointcloud_host_;
  device_vector<Vector3f> lidar_pointcloud_device_;
};

}  // namespace nvblox

#include "nvblox_ros/impl/conversions_impl.hpp"

#endif  // NVBLOX_ROS__CONVERSIONS_HPP_
