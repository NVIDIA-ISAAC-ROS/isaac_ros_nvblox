// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVBLOX_ROS__CONVERSIONS__ESDF_AND_GRADIENTS_CONVERSIONS_HPP_
#define NVBLOX_ROS__CONVERSIONS__ESDF_AND_GRADIENTS_CONVERSIONS_HPP_

#include <nvblox/nvblox.h>

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>

#include <nvblox_msgs/srv/esdf_and_gradients.hpp>

namespace nvblox
{
namespace conversions
{

class EsdfAndGradientsConverter
{
public:
  EsdfAndGradientsConverter()
  : gpu_grid_(MemoryType::kDevice), cpu_grid_(MemoryType::kHost) {}
  ~EsdfAndGradientsConverter() = default;

  /// @brief Returns the response to a EsdfAndGradients request.
  /// @param esdf_layer The layer used to fill the EsdfAndGradients grid.
  /// @param default_value The value array entry should take where the layer has
  /// no observation.
  /// @param request Pointer to request defining the AABB.
  /// @param response Pointer to the response we provide.
  /// @param cuda_stream  The stream to do the conversion on.
  void getEsdfAndGradientResponse(
    const EsdfLayer & esdf_layer,          // NOLINT
    const float default_value,             // NOLINT
    const std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Request> request,
    std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Response> response,
    const CudaStream & cuda_stream);

  /// Converts an ESDF Layer within an AABB to a ROS array message.
  /// @param esdf_layer The layer to convert.
  /// @param aabb The AABB describing the region to be converted. The output
  /// array includes the aabb end points.
  /// @param default_value The value array entry should take where the layer has
  /// no observation.
  /// @param cuda_stream The stream to do the conversion on.
  /// @return The ROS message.
  std_msgs::msg::Float32MultiArray esdfInAabbToMultiArrayMsg(
    const EsdfLayer & esdf_layer,          // NOLINT
    const AxisAlignedBoundingBox & aabb,   // NOLINT
    const float default_value,             // NOLINT
    const CudaStream & cuda_stream);

protected:
  // Staging space on the device
  Unified3DGrid<float> gpu_grid_;
  Unified3DGrid<float> cpu_grid_;
};

/// @brief Get the list of shapes to clear from the EsdfAndGradients service request.
/// @param request Pointer to request defining the shapes.
/// @param logger ROS logger.
/// @return List of shapes that should be cleared.
std::vector<BoundingShape> getShapesToClear(
  const std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Request> request,
  rclcpp::Logger logger);

}  // namespace conversions
}  // namespace nvblox

#endif  // NVBLOX_ROS__CONVERSIONS__ESDF_AND_GRADIENTS_CONVERSIONS_HPP_
