/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <glog/logging.h>
#include <nvblox/core/bounding_boxes.h>

#include <string>
#include <vector>

#include "nvblox_ros/conversions.hpp"

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
  nvblox_msgs::msg::Mesh * mesh_msg)
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

}  // namespace nvblox
