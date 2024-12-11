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
#include "nvblox_ros/conversions/esdf_and_gradients_conversions.hpp"

#include <nvblox/core/types.h>
#include <nvblox/map/unified_3d_grid.h>
#include <nvblox/map/voxels.h>
#include <nvblox/map/internal/cuda/impl/layer_to_3d_grid_impl.cuh>
#include <nvblox/map/internal/cuda/layer_to_3d_grid.cuh>

namespace nvblox {
namespace conversions {

struct SignedDistanceFunctor {
  SignedDistanceFunctor(float _voxel_size, float _default_value)
      : voxel_size(_voxel_size), default_value(_default_value) {}
  ~SignedDistanceFunctor() = default;

  __device__ __inline__ float operator()(const EsdfVoxel& esdf_voxel) const {
    float distance_m = default_value;
    if (esdf_voxel.observed) {
      assert(esdf_voxel.squared_distance_vox >= 0.0f);
      const float unsigned_distance_vox = sqrt(esdf_voxel.squared_distance_vox);
      distance_m = unsigned_distance_vox * voxel_size;
      if (esdf_voxel.is_inside) {
        distance_m *= -1.0f;
      }
    }
    return distance_m;
  }

  const float voxel_size;
  const float default_value;
};

void EsdfAndGradientsConverter::getEsdfAndGradientResponse(
    const EsdfLayer& esdf_layer,
    const float default_value,
    const std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Request> request,
    std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Response> response,
    const CudaStream& cuda_stream){
      // Construct the bounding box.
      AxisAlignedBoundingBox aabb;
      if (request->use_aabb){
        const Vector3f aabb_min_m(request->aabb_min_m.x, request->aabb_min_m.y,
          request->aabb_min_m.z);
        const Vector3f aabb_size_m(request->aabb_size_m.x, request->aabb_size_m.y,
          request->aabb_size_m.z);
        aabb = AxisAlignedBoundingBox(aabb_min_m, aabb_min_m + aabb_size_m);
      } else {
        aabb = getAABBOfAllocatedBlocks(esdf_layer);
      }

      if (aabb.isEmpty()){
        response->success = false;
      } else {
        // Convert the layer to a message.
        response->voxel_size_m = esdf_layer.voxel_size();
        response->esdf_and_gradients =
          esdfInAabbToMultiArrayMsg(
          esdf_layer, aabb,
          default_value, cuda_stream);
    
        // The origin is the minimal corner of the minimal voxel in the grid.
        Vector3f origin_m = cpu_grid_.min_index().cast<float>() * esdf_layer.voxel_size();
        response->origin_m.x = origin_m.x();
        response->origin_m.y = origin_m.y();
        response->origin_m.z = origin_m.z();

        response->success = true;
      }
}

std_msgs::msg::Float32MultiArray
EsdfAndGradientsConverter::esdfInAabbToMultiArrayMsg(
    const EsdfLayer& esdf_layer,         // NOLINT
    const AxisAlignedBoundingBox& aabb,  // NOLINT
    const float default_value,           // NOLINT
    const CudaStream& cuda_stream) {
  // Copy the values out to the grid on the device
  SignedDistanceFunctor conversion_op(esdf_layer.voxel_size(), default_value);
  voxelLayerToDenseVoxelGridInAABBAsync(esdf_layer, aabb, default_value,
                                        conversion_op, &gpu_grid_, cuda_stream);

  // GPU grid -> CPU (pinned) grid
  cpu_grid_.copyFromAsync(gpu_grid_, cuda_stream);

  // The size in voxels
  const Index3D size_in_voxels = gpu_grid_.aabb_size();

  // CPU grid -> ROS message
  std_msgs::msg::Float32MultiArray array_msg;
  array_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  array_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  array_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  array_msg.layout.dim[0].label = "x";
  array_msg.layout.dim[0].size = size_in_voxels.x();
  array_msg.layout.dim[0].stride =
      size_in_voxels.x() * size_in_voxels.y() * size_in_voxels.z();
  array_msg.layout.dim[1].label = "y";
  array_msg.layout.dim[1].size = size_in_voxels.y();
  array_msg.layout.dim[1].stride = size_in_voxels.y() * size_in_voxels.z();
  array_msg.layout.dim[2].label = "z";
  array_msg.layout.dim[2].size = size_in_voxels.z();
  array_msg.layout.dim[2].stride = size_in_voxels.z();
  array_msg.data = cpu_grid_.data().toVectorAsync(cuda_stream);

  cuda_stream.synchronize();

  return array_msg;
}

std::vector<BoundingShape> getShapesToClear(
    const std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Request> request,
    rclcpp::Logger logger) {
  std::vector<BoundingShape> shapes_to_clear;

  // Add AABBs to shapes_to_clear vector.
  const size_t number_of_aabbs = request->aabbs_to_clear_min_m.size();
  if (number_of_aabbs == request->aabbs_to_clear_size_m.size()) {
    for (size_t i = 0; i < number_of_aabbs; i++) {
      // Get the min and size of the bounding box from the request.
      const geometry_msgs::msg::Point aabb_min_msg =
          request->aabbs_to_clear_min_m[i];
      const geometry_msgs::msg::Vector3 aabb_size_msg =
          request->aabbs_to_clear_size_m[i];
      // Convert to an AABB.
      const Vector3f aabb_min(aabb_min_msg.x, aabb_min_msg.y, aabb_min_msg.z);
      const Vector3f aabb_size(aabb_size_msg.x, aabb_size_msg.y,
                               aabb_size_msg.z);
      AxisAlignedBoundingBox aabb(aabb_min, aabb_min + aabb_size);
      // Add the AABB to the shape vector.
      if (!aabb.isEmpty()) {
        shapes_to_clear.push_back(BoundingShape(aabb));
      }
    }
  } else {
    RCLCPP_WARN_STREAM(
        logger,
        "Sizes of clearing aabb vectors do not match. Not clearing the AABBs.");
  }

  // Add spheres to shapes_to_clear vector.
  const size_t number_of_spheres = request->spheres_to_clear_center_m.size();
  if (number_of_spheres == request->spheres_to_clear_radius_m.size()) {
    for (size_t i = 0; i < number_of_spheres; i++) {
      // Get the center and radius of the sphere from the request.
      const geometry_msgs::msg::Point sphere_center_msg =
          request->spheres_to_clear_center_m[i];
      const float sphere_radius = request->spheres_to_clear_radius_m[i];
      // Convert to a BoundingSphere.
      const Vector3f sphere_center(sphere_center_msg.x, sphere_center_msg.y,
                                   sphere_center_msg.z);
      BoundingSphere sphere(sphere_center, sphere_radius);
      if (sphere.radius() > 0.f) {
        // Add the sphere to the shape vector.
        shapes_to_clear.push_back(BoundingShape(sphere));
      }
    }
  } else {
    RCLCPP_WARN_STREAM(logger,
                       "Sizes of clearing sphere vectors do not match. Not "
                       "clearing the spheres.");
  }
  return shapes_to_clear;
}

}  // namespace conversions
}  // namespace nvblox
