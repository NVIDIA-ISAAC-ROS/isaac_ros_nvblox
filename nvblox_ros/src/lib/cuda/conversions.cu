/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <nvblox/core/bounding_boxes.h>
#include <nvblox/core/unified_vector.h>
#include <nvblox/gpu_hash/cuda/gpu_hash_interface.cuh>
#include <nvblox/gpu_hash/cuda/gpu_indexing.cuh>

#include <thrust/functional.h>
#include <thrust/transform.h>

#include "nvblox_ros/conversions.hpp"

namespace nvblox {

template <typename VoxelType>
__device__ bool getVoxelIntensity(const VoxelType& voxel, float voxel_size,
                                  float* intensity);

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

// Inputs: GPU hash for the E/TSDF.
//         AABB.
//         Voxel Size (just needed for ESDF).
// Outputs: vector of pcl::PointXYZIs.
//          max index (updated atomically).
template <typename VoxelType>
__global__ void copyPointcloudToPCL(
    Index3DDeviceHashMapType<VoxelBlock<VoxelType>> block_hash,
    Index3D* block_indices, size_t num_indices, int max_output_indices,
    AxisAlignedBoundingBox aabb, float block_size, PclPoint* pointcloud,
    int* max_index) {
  const float voxel_size = block_size / VoxelBlock<VoxelType>::kVoxelsPerSide;

  // Get the relevant block.
  __shared__ VoxelBlock<VoxelType>* block_ptr;
  if (threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0) {
    block_ptr = nullptr;
    auto it = block_hash.find(block_indices[blockIdx.x]);
    if (it != block_hash.end()) {
      block_ptr = it->second;
    } else {
      return;
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
  if (!getVoxelIntensity<VoxelType>(voxel, voxel_size, &intensity)) {
    return;
  }

  // Otherwise shove it in the output.
  int next_index = atomicAdd(max_index, 1);
  if (next_index >= max_output_indices) {
    printf("Overrunning the space. This shouldn't happen.\n");
    return;
  }
  PclPoint& point = pointcloud[next_index];
  point.x = voxel_position.x();
  point.y = voxel_position.y();
  point.z = voxel_position.z();
  point.intensity = intensity;
}

template <typename VoxelType>
void RosConverter::convertLayerInAABBToPCLCuda(
    const VoxelBlockLayer<VoxelType>& layer, const AxisAlignedBoundingBox& aabb,
    sensor_msgs::msg::PointCloud2* pointcloud) {
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
  block_indices_device_ = block_indices;

  if (block_indices.empty()) {
    return;
  }
  size_t num_voxels = block_indices.size() * kVoxelsPerBlock;

  // Allocate a GPU pointcloud.
  pointcloud_device_.reserve(num_voxels);

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

  copyPointcloudToPCL<VoxelType><<<dim_block, dim_threads, 0, cuda_stream_>>>(
      gpu_layer_view.getHash().impl_, block_indices_device_.data(),
      block_indices.size(), num_voxels, aabb_intersect, layer.block_size(),
      pointcloud_device_.data(), max_index_device_.get());
  checkCudaErrors(cudaStreamSynchronize(cuda_stream_));
  checkCudaErrors(cudaPeekAtLastError());

  // Copy the pointcloud out.
  max_index_host_ = max_index_device_.clone(MemoryType::kHost);

  size_t output_size = sizeof(PclPoint) * *max_index_host_;
  pointcloud->data.resize(output_size);
  // Copy over all the points.
  cudaMemcpy(pointcloud->data.data(), pointcloud_device_.data(), output_size,
             cudaMemcpyDeviceToHost);

  // Fill the other fields in the pointcloud message.
  pointcloud->height = 1;
  pointcloud->width = *max_index_host_;
  pointcloud->point_step = sizeof(PclPoint);
  pointcloud->row_step = output_size;

  // Populate the fields.
  sensor_msgs::msg::PointField point_field;
  point_field.name = "x";
  point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_field.offset = 0;
  point_field.count = 1;

  pointcloud->fields.push_back(point_field);
  point_field.name = "y";
  point_field.offset += sizeof(float);
  pointcloud->fields.push_back(point_field);
  point_field.name = "z";
  point_field.offset += sizeof(float);
  pointcloud->fields.push_back(point_field);
  point_field.name = "intensity";
  point_field.offset += sizeof(float);
  pointcloud->fields.push_back(point_field);
}

// Template specializations.
template void RosConverter::convertLayerInAABBToPCLCuda<TsdfVoxel>(
    const VoxelBlockLayer<TsdfVoxel>& layer, const AxisAlignedBoundingBox& aabb,
    sensor_msgs::msg::PointCloud2* pointcloud);

template void RosConverter::convertLayerInAABBToPCLCuda<EsdfVoxel>(
    const VoxelBlockLayer<EsdfVoxel>& layer, const AxisAlignedBoundingBox& aabb,
    sensor_msgs::msg::PointCloud2* pointcloud);

void RosConverter::meshBlockMessageFromMeshBlock(
    const MeshBlock& mesh_block, nvblox_msgs::msg::MeshBlock* mesh_block_msg) {
  CHECK_NOTNULL(mesh_block_msg);

  size_t num_vertices = mesh_block.vertices.size();

  mesh_block_msg->vertices.resize(num_vertices);
  mesh_block_msg->normals.resize(num_vertices);
  mesh_block_msg->colors.resize(mesh_block.colors.size());
  mesh_block_msg->triangles.resize(mesh_block.triangles.size());

  std::vector<Vector3f> vertices = mesh_block.getVertexVectorOnCPU();
  std::vector<Vector3f> normals = mesh_block.getNormalVectorOnCPU();
  std::vector<Color> colors = mesh_block.getColorVectorOnCPU();

  // Copy over vertices and normals.
  for (size_t i = 0; i < num_vertices; i++) {
    mesh_block_msg->vertices[i] = pointMessageFromVector(vertices[i]);
    mesh_block_msg->normals[i] = pointMessageFromVector(normals[i]);
  }

  // Copy over colors if available.
  for (size_t i = 0; i < mesh_block.colors.size(); i++) {
    mesh_block_msg->colors[i] = colorMessageFromColor(colors[i]);
  }

  // Copying over triangles is thankfully easy.
  mesh_block_msg->triangles = mesh_block.getTriangleVectorOnCPU();
}

struct DivideBy1000 : public thrust::unary_function<uint16_t, float> {
  __host__ __device__ float operator()(const uint16_t& in) {
    return static_cast<float>(in) / 1000.0f;
  }
};

// Convert image to depth frame object
bool RosConverter::depthImageFromImageMessage(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    DepthImage* depth_image) {
  CHECK_NOTNULL(depth_image);
  // If the image is a float, we can just copy it over directly.
  // If the image is int16, we need to divide by 1000 to get the correct
  // format for us.

  // First check if we actually have a valid image here.
  if (image_msg->encoding != "32FC1" && image_msg->encoding != "16UC1") {
    return false;
  }

  // Fill it in. How this is done depends on what the image encoding is.
  if (image_msg->encoding == "32FC1") {
    // Float to float, so this should be a straight-up copy. :)
    depth_image->populateFromBuffer(
        image_msg->height, image_msg->width,
        reinterpret_cast<const float*>(&image_msg->data[0]));
  } else if (image_msg->encoding == "16UC1") {
    // Then we have to just go byte-by-byte and convert this. This is a massive
    // pain and slow. We need to find a better way to do this; on GPU or
    // through openCV.
    const uint16_t* char_depth_buffer =
        reinterpret_cast<const uint16_t*>(&image_msg->data[0]);
    const int numel = image_msg->height * image_msg->width;

    bool kUseCuda = false;
    if (kUseCuda) {
      // Make sure there's enough output space.
      if (depth_image->numel() < numel) {
        *depth_image = DepthImage(image_msg->height, image_msg->width,
                                  MemoryType::kDevice);
      }

      // Now just thrust it.
      thrust::transform(char_depth_buffer, char_depth_buffer + numel,
                        depth_image->dataPtr(), DivideBy1000());
    } else {
      std::vector<float> float_depth_buffer(numel);
      for (int i = 0; i < numel; i++) {
        float_depth_buffer[i] =
            static_cast<float>(char_depth_buffer[i]) / 1000.0f;
      }
      depth_image->populateFromBuffer(image_msg->height, image_msg->width,
                                      float_depth_buffer.data(),
                                      MemoryType::kDevice);
    }
  }

  return true;
}

__global__ void populateSliceFromLayerKernel(
    Index3DDeviceHashMapType<EsdfBlock> block_hash, AxisAlignedBoundingBox aabb,
    float block_size, float* image, int rows, int cols, float z_slice_height,
    float resolution, float unobserved_value) {
  const float voxel_size = block_size / EsdfBlock::kVoxelsPerSide;
  const int pixel_col = blockIdx.x * blockDim.x + threadIdx.x;
  const int pixel_row = blockIdx.y * blockDim.y + threadIdx.y;

  if (pixel_col >= cols || pixel_row >= rows) {
    return;
  }

  // Figure out where this pixel should map to.
  Vector3f voxel_position(aabb.min().x() + resolution * pixel_col,
                          aabb.min().y() + resolution * pixel_row,
                          z_slice_height);

  Index3D block_index, voxel_index;

  getBlockAndVoxelIndexFromPositionInLayer(block_size, voxel_position,
                                           &block_index, &voxel_index);

  // Get the relevant block.
  EsdfBlock* block_ptr = nullptr;
  auto it = block_hash.find(block_index);
  if (it != block_hash.end()) {
    block_ptr = it->second;
  } else {
    image::access(pixel_row, pixel_col, cols, image) = unobserved_value;
    return;
  }

  // Get the relevant pixel.
  const EsdfVoxel* voxel =
      &block_ptr->voxels[voxel_index.x()][voxel_index.y()][voxel_index.z()];
  float distance = unobserved_value;
  if (voxel->observed) {
    distance = voxel_size * std::sqrt(voxel->squared_distance_vox);
    if (voxel->is_inside) {
      distance = -distance;
    }
  }
  image::access(pixel_row, pixel_col, cols, image) = distance;
}

void RosConverter::populateSliceFromLayer(const EsdfLayer& layer,
                                          const AxisAlignedBoundingBox& aabb,
                                          float z_slice_height,
                                          float resolution,
                                          float unobserved_value,
                                          Image<float>* image) {
  if (image->numel() <= 0) {
    return;
  }
  const float voxel_size = layer.voxel_size();

  // Create a GPU hash of the ESDF.
  GPULayerView<EsdfBlock> gpu_layer_view = layer.getGpuLayerView();

  // Pass in the GPU hash and AABB and let the kernel figure it out.
  constexpr int kThreadDim = 16;
  const int rounded_rows = static_cast<int>(
      std::ceil(image->rows() / static_cast<float>(kThreadDim)));
  const int rounded_cols = static_cast<int>(
      std::ceil(image->cols() / static_cast<float>(kThreadDim)));
  dim3 block_dim(rounded_cols, rounded_rows);
  dim3 thread_dim(kThreadDim, kThreadDim);

  populateSliceFromLayerKernel<<<block_dim, thread_dim, 0, cuda_stream_>>>(
      gpu_layer_view.getHash().impl_, aabb, layer.block_size(),
      image->dataPtr(), image->rows(), image->cols(), z_slice_height,
      resolution, unobserved_value);
  checkCudaErrors(cudaStreamSynchronize(cuda_stream_));
  checkCudaErrors(cudaPeekAtLastError());
}

}  // namespace nvblox
