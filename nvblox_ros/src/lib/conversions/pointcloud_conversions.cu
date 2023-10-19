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

#include <thrust/execution_policy.h>
#include <thrust/transform.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "nvblox_ros/conversions/pointcloud_conversions.hpp"

namespace nvblox {
namespace conversions {


void copyDevicePointcloudToMsg(
    const device_vector<PclPointXYZI>& pcl_pointcloud_device,
    sensor_msgs::msg::PointCloud2* pointcloud_msg) {
  // Copy into the pointcloud message.
  const int num_points = pcl_pointcloud_device.size();
  size_t output_num_bytes = sizeof(PclPointXYZI) * num_points;
  pointcloud_msg->data.resize(output_num_bytes);
  // Copy over all the points.
  cudaMemcpy(pointcloud_msg->data.data(), pcl_pointcloud_device.data(),
             output_num_bytes, cudaMemcpyDeviceToHost);

  // Fill the other fields in the pointcloud_msg message.
  pointcloud_msg->height = 1;
  pointcloud_msg->width = num_points;
  pointcloud_msg->point_step = sizeof(PclPointXYZI);
  pointcloud_msg->row_step = output_num_bytes;

  // Populate the fields.
  sensor_msgs::msg::PointField point_field;
  point_field.name = "x";
  point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_field.offset = 0;
  point_field.count = 1;

  pointcloud_msg->fields.push_back(point_field);
  point_field.name = "y";
  point_field.offset += sizeof(float);
  pointcloud_msg->fields.push_back(point_field);
  point_field.name = "z";
  point_field.offset += sizeof(float);
  pointcloud_msg->fields.push_back(point_field);
  point_field.name = "intensity";
  point_field.offset += sizeof(float);
  pointcloud_msg->fields.push_back(point_field);
}


PointcloudConverter::PointcloudConverter()
    : PointcloudConverter(std::make_shared<CudaStreamOwning>()) {}

PointcloudConverter::PointcloudConverter(std::shared_ptr<CudaStream> cuda_stream)
    : cuda_stream_(cuda_stream) {}


bool PointcloudConverter::checkLidarPointcloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud,
  const Lidar & lidar)
{
  // Check the cache
  if (checked_lidar_models_.find(lidar) != checked_lidar_models_.end()) {
    return true;
  }

  // Go through the pointcloud and check that each point projects to a pixel
  // center.
  sensor_msgs::PointCloud2ConstIterator<float> iter_xyz(*pointcloud, "x");
  for (; iter_xyz != iter_xyz.end(); ++iter_xyz) {
    Vector3f point(iter_xyz[0], iter_xyz[1], iter_xyz[2]);
    if (point.hasNaN()) {
      continue;
    }
    Vector2f u_C;
    if (!lidar.project(point, &u_C)) {
      // Point fell outside the FoV specified in the intrinsics.
      return false;
    }
  }
  checked_lidar_models_.insert(lidar);
  return true;
}

void PointcloudConverter::writeLidarPointcloudToFile(
  const std::string filepath_prefix,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud)
{
  // Write the dimensions
  std::ofstream width_file(filepath_prefix + "_dims.txt", std::ofstream::out);
  width_file << pointcloud->width << ", " << pointcloud->height;
  width_file.close();
  // Write the data
  Eigen::MatrixX3f pointcloud_matrix(pointcloud->width * pointcloud->height, 3);
  sensor_msgs::PointCloud2ConstIterator<float> iter_xyz(*pointcloud, "x");
  int idx = 0;
  for (; iter_xyz != iter_xyz.end(); ++iter_xyz) {
    Vector3f point(iter_xyz[0], iter_xyz[1], iter_xyz[2]);
    pointcloud_matrix.row(idx) = point;
    idx++;
  }
  io::writeToCsv(filepath_prefix + ".csv", pointcloud_matrix);
}


__global__ void depthImageFromPointcloudKernel(
    const Vector3f* pointcloud,          // NOLINT
    const Lidar lidar,                   // NOLINT
    const int num_bytes_between_points,  // NOLINT
    const int num_points,                // NOLINT
    float* depth_image) {
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;

  if (idx >= num_points) {
    return;
  }

  // Read a point from global memory
  const Vector3f point = pointcloud[idx];

  if (isnan(point.x()) || isnan(point.y()) || isnan(point.z())) {
    return;
  }

  // Project
  Index2D u_C_int;
  if (!lidar.project(point, &u_C_int)) {
    return;
  }

  // Write the depth to the image
  // NOTE(alexmillane): It's possible multiple points project to the same pixel.
  // Firstly, writes should be atomic, so we're fine in that respect. The last
  // projection to write "wins" so we basically end up with a random value, of
  // the values that project to this pixel.
  image::access(u_C_int.y(), u_C_int.x(), lidar.num_azimuth_divisions(),
                depth_image) = lidar.getDepth(point);
}

void PointcloudConverter::depthImageFromPointcloudGPU(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud,
    const Lidar& lidar, DepthImage* depth_image_ptr) {
  CHECK(depth_image_ptr->memory_type() == MemoryType::kDevice ||
        depth_image_ptr->memory_type() == MemoryType::kUnified);

  // Check output space, and reallocate if required
  if ((depth_image_ptr->rows() != lidar.num_elevation_divisions()) ||
      (depth_image_ptr->cols() != lidar.num_azimuth_divisions())) {
    *depth_image_ptr =
        DepthImage(lidar.num_elevation_divisions(),
                   lidar.num_azimuth_divisions(), MemoryType::kDevice);
  }

  // Set the entire image to 0.
  depth_image_ptr->setZero();

  // Get a pointer to the pointcloud
  sensor_msgs::PointCloud2ConstIterator<float> iter_xyz(*pointcloud, "x");
  const float* pointcloud_data_ptr = &(*iter_xyz);
  const int num_bytes_between_points = pointcloud->point_step;
  const int num_points = pointcloud->width * pointcloud->height;

  // Expand buffers where required
  if (lidar.numel() > lidar_pointcloud_host_.capacity()) {
    const int new_size = static_cast<int>(lidar.numel());
    lidar_pointcloud_host_.reserve(new_size);
    lidar_pointcloud_device_.reserve(new_size);
  }

  // Copy the pointcloud into pinned host memory
  lidar_pointcloud_host_.clear();
  for (; iter_xyz != iter_xyz.end(); ++iter_xyz) {
    lidar_pointcloud_host_.push_back(
        Vector3f(iter_xyz[0], iter_xyz[1], iter_xyz[2]));
  }
  // Copy the pointcloud to the GPU
  lidar_pointcloud_device_.copyFromAsync(lidar_pointcloud_host_, *cuda_stream_);

  // Convert to an image on the GPU
  constexpr int num_threads_per_block = 256;  // because why not
  const int num_thread_blocks = lidar.numel() / num_threads_per_block + 1;
  depthImageFromPointcloudKernel<<<num_thread_blocks, num_threads_per_block, 0,
                                   *cuda_stream_>>>(
      lidar_pointcloud_device_.data(), lidar, num_bytes_between_points,
      lidar.numel(), depth_image_ptr->dataPtr());
  checkCudaErrors(cudaStreamSynchronize(*cuda_stream_));
  checkCudaErrors(cudaPeekAtLastError());
}

struct Vector3fToPcl {
  __host__ __device__ PclPointXYZI operator()(const Vector3f& vec) const {
    PclPointXYZI point;
    point.x = vec.x();
    point.y = vec.y();
    point.z = vec.z();
    point.intensity = 1.0;
    return point;
  };
};

// Internal pointcloud representation to a ROS pointcloud
void PointcloudConverter::pointcloudMsgFromPointcloud(
    const Pointcloud& pointcloud,
    sensor_msgs::msg::PointCloud2* pointcloud_msg) {
  CHECK_NOTNULL(pointcloud_msg);
  CHECK(pointcloud.memory_type() == MemoryType::kDevice ||
        pointcloud.memory_type() == MemoryType::kUnified);

  pcl_pointcloud_device_.resize(pointcloud.size());

  thrust::transform(thrust::device, pointcloud.points().begin(),
                    pointcloud.points().end(), pcl_pointcloud_device_.begin(),
                    Vector3fToPcl());

  // Put the points in the message
  copyDevicePointcloudToMsg(pcl_pointcloud_device_, pointcloud_msg);
}

void PointcloudConverter::pointsToCubesMarkerMsg(
    const std::vector<Vector3f>& points, const float cube_size,
    const Color& color, visualization_msgs::msg::Marker* marker_ptr) {
  CHECK_NOTNULL(marker_ptr);
  // Publish
  marker_ptr->action = marker_ptr->ADD;
  marker_ptr->type = marker_ptr->CUBE_LIST;
  marker_ptr->points.reserve(points.size());
  for (const Vector3f& p_L : points) {
    geometry_msgs::msg::Point point;
    point.x = p_L.x();
    point.y = p_L.y();
    point.z = p_L.z();
    marker_ptr->points.push_back(point);
  }
  marker_ptr->color.r = static_cast<float>(color.r) / 255.0f;
  marker_ptr->color.g = static_cast<float>(color.g) / 255.0f;
  marker_ptr->color.b = static_cast<float>(color.b) / 255.0f;
  marker_ptr->color.a = static_cast<float>(color.a) / 255.0f;
  marker_ptr->scale.x = cube_size;
  marker_ptr->scale.y = cube_size;
  marker_ptr->scale.z = cube_size;
}

}  // namespace conversions
}  // namespace nvblox
