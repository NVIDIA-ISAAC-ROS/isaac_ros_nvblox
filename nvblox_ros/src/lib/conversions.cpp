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

#include "nvblox_ros/conversions.hpp"

#include <glog/logging.h>

#include <nvblox/core/bounding_boxes.h>

#include <string>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace nvblox
{

RosConverter::RosConverter() {cudaStreamCreate(&cuda_stream_);}

// Convert camera info message to NVBlox camera object
Camera RosConverter::cameraFromMessage(
  const sensor_msgs::msg::CameraInfo & camera_info)
{
  Camera camera(camera_info.k[0], camera_info.k[4], camera_info.k[2],
    camera_info.k[5], camera_info.width, camera_info.height);
  return camera;
}

// Convert image to depth frame object
bool RosConverter::colorImageFromImageMessage(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  ColorImage * color_image)
{
  CHECK_NOTNULL(color_image);

  // First check if we actually have a valid image here.
  if (image_msg->encoding != "rgb8") {
    return false;
  }

  color_image->populateFromBuffer(
    image_msg->height, image_msg->width,
    reinterpret_cast<const Color *>(&image_msg->data[0]), MemoryType::kDevice);

  return true;
}

void RosConverter::imageMessageFromDepthImage(
  const DepthImage & depth_image, const std::string & frame_id,
  sensor_msgs::msg::Image * image_msg)
{
  CHECK_NOTNULL(image_msg);
  size_t image_size =
    depth_image.width() * depth_image.height() * sizeof(float);
  image_msg->data.resize(image_size);

  image_msg->header.frame_id = frame_id;
  image_msg->width = depth_image.width();
  image_msg->height = depth_image.height();
  image_msg->step = depth_image.width() * sizeof(float);

  image_msg->encoding = "32FC1";

  cudaMemcpy(
    &image_msg->data[0], depth_image.dataConstPtr(), image_size,
    cudaMemcpyDefault);
}

void RosConverter::meshMessageFromMeshLayer(
  const BlockLayer<MeshBlock> & mesh_layer, nvblox_msgs::msg::Mesh * mesh_msg)
{
  std::vector<Index3D> block_indices = mesh_layer.getAllBlockIndices();
  meshMessageFromMeshBlocks(mesh_layer, block_indices, mesh_msg);
}

void RosConverter::meshMessageFromMeshBlocks(
  const BlockLayer<MeshBlock> & mesh_layer,
  const std::vector<Index3D> & block_indices,
  nvblox_msgs::msg::Mesh * mesh_msg,
  const std::vector<Index3D> & block_indices_to_delete)
{
  // Go through all the blocks, converting each individual one.
  mesh_msg->block_size = mesh_layer.block_size();
  mesh_msg->block_indices.resize(block_indices.size());
  mesh_msg->blocks.resize(block_indices.size());

  for (size_t i = 0; i < block_indices.size(); i++) {
    // Get the block origin.
    mesh_msg->block_indices[i] = index3DMessageFromIndex3D(block_indices[i]);

    MeshBlock::ConstPtr mesh_block =
      mesh_layer.getBlockAtIndex(block_indices[i]);
    if (mesh_block == nullptr) {
      continue;
    }

    // Convert the actual block.
    meshBlockMessageFromMeshBlock(*mesh_block, &mesh_msg->blocks[i]);
  }

  for (const Index3D & block_index : block_indices_to_delete) {
    mesh_msg->block_indices.push_back(index3DMessageFromIndex3D(block_index));
    mesh_msg->blocks.push_back(nvblox_msgs::msg::MeshBlock());
  }
}

void RosConverter::distanceMapSliceFromLayer(
  const EsdfLayer & layer, float z_slice_level,
  nvblox_msgs::msg::DistanceMapSlice * map_slice)
{
  CHECK_NOTNULL(map_slice);

  // Unlikely we're going to have anything be 1000 meters inside an obstacle.
  float kUnknownValue = -1000.0f;

  // Figure out how big the map needs to be.
  // To do this, we get all the block indices, figure out which intersect
  // with our desired height, and select the min and max in the X and Y
  // direction.
  const float block_size = layer.block_size();
  constexpr int kVoxelsPerSide = VoxelBlock<EsdfVoxel>::kVoxelsPerSide;
  const float voxel_size = block_size / kVoxelsPerSide;
  std::vector<Index3D> block_indices = layer.getAllBlockIndices();
  AxisAlignedBoundingBox aabb;
  aabb.setEmpty();

  // Figure out the index of the desired height.
  Index3D desired_z_block_index;
  Index3D desired_z_voxel_index;
  getBlockAndVoxelIndexFromPositionInLayer(
    block_size, Vector3f(0.0f, 0.0f, z_slice_level), &desired_z_block_index,
    &desired_z_voxel_index);

  for (const Index3D & block_index : block_indices) {
    // Skip all other heights of block.
    if (block_index.z() != desired_z_block_index.z()) {
      continue;
    }

    // Extend the AABB by the dimensions of this block.
    aabb.extend(getAABBOfBlock(block_size, block_index));
  }

  Vector3f bounding_size = aabb.sizes();
  // Width = cols, height = rows
  int width = static_cast<int>(std::ceil(bounding_size.x() / voxel_size));
  int height = static_cast<int>(std::ceil(bounding_size.y() / voxel_size));

  int map_size = width * height;

  // Set all the other settings in the message.
  map_slice->origin.x = aabb.min().x();
  map_slice->origin.y = aabb.min().y();
  map_slice->origin.z = z_slice_level;

  map_slice->resolution = voxel_size;
  map_slice->width = width;
  map_slice->height = height;
  map_slice->unknown_value = kUnknownValue;

  // Allocate the map directly, we will write directly to this output to prevent
  // copies.
  map_slice->data.resize(map_size, kUnknownValue);

  // Create an image of the correct size.
  Image<float> image(height, width, MemoryType::kDevice);

  // Fill in the float image.
  populateSliceFromLayer(
    layer, aabb, z_slice_level, voxel_size, kUnknownValue,
    &image);

  checkCudaErrors(
    cudaMemcpy(
      map_slice->data.data(), image.dataPtr(),
      image.numel() * sizeof(float), cudaMemcpyDefault));
}

// Convert a mesh to a marker array.
void RosConverter::markerMessageFromMeshLayer(
  const BlockLayer<MeshBlock> & mesh_layer, const std::string & frame_id,
  visualization_msgs::msg::MarkerArray * marker_msg)
{
  // Get all the mesh blocks.
  std::vector<Index3D> indices = mesh_layer.getAllBlockIndices();

  marker_msg->markers.resize(indices.size());

  size_t output_index = 0;
  for (size_t i = 0; i < indices.size(); i++) {
    MeshBlock::ConstPtr mesh_block = mesh_layer.getBlockAtIndex(indices[i]);
    if (mesh_block->size() == 0) {
      continue;
    }
    markerMessageFromMeshBlock(
      mesh_block, frame_id,
      &marker_msg->markers[output_index]);
    marker_msg->markers[output_index].id = output_index;
    std::stringstream ns_stream;
    ns_stream << indices[i].x() << "_" << indices[i].y() << "_" <<
      indices[i].z();
    marker_msg->markers[output_index].ns = ns_stream.str();
    output_index++;
  }
  marker_msg->markers.resize(output_index);
}

void RosConverter::markerMessageFromMeshBlock(
  const MeshBlock::ConstPtr & mesh_block, const std::string & frame_id,
  visualization_msgs::msg::Marker * marker)
{
  marker->header.frame_id = frame_id;
  marker->ns = "mesh";
  marker->scale.x = 1;
  marker->scale.y = 1;
  marker->scale.z = 1;
  marker->pose.orientation.x = 0;
  marker->pose.orientation.y = 0;
  marker->pose.orientation.z = 0;
  marker->pose.orientation.w = 1;
  marker->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;

  // Assumes UNWELDED mesh: all vertices in order.
  std::vector<Vector3f> vertices = mesh_block->getVertexVectorOnCPU();
  std::vector<Color> colors = mesh_block->getColorVectorOnCPU();
  std::vector<int> triangles = mesh_block->getTriangleVectorOnCPU();

  CHECK_EQ(vertices.size(), colors.size());

  marker->points.reserve(triangles.size());
  marker->colors.reserve(triangles.size());

  for (size_t i = 0; i < triangles.size(); i++) {
    int index = triangles[i];
    if (index >= colors.size() || index >= vertices.size()) {
      continue;
    }
    marker->points.push_back(pointMessageFromVector(vertices[index]));
    marker->colors.push_back(colorMessageFromColor(colors[index]));
  }
}

bool RosConverter::checkLidarPointcloud(
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

void RosConverter::writeLidarPointcloudToFile(
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

}  // namespace nvblox
