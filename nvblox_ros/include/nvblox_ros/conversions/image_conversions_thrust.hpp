// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVBLOX_ROS__CONVERSIONS__IMAGE_CONVERSIONS_THRUST_HPP_
#define NVBLOX_ROS__CONVERSIONS__IMAGE_CONVERSIONS_THRUST_HPP_

#include <nvblox/nvblox.h>

namespace nvblox
{
namespace conversions
{

using Rgb = std::array<uint8_t, 3>;
using Bgra = std::array<uint8_t, 4>;
using Rgba = nvblox::Color;

/// Convert device RGB/BGRA -> device RGBA (with alpha = 255)
/// Output image is resized if necessary
template<typename T>
bool rgbaFromDeviceAsync(
  const T * ptr_device, const int height,
  const int width, ColorImage * color_image,
  const CudaStream & cuda_stream);

/// Convert host RGB/BGRA -> device RGBA (with alpha = 255)
/// Output and tmp images are resized if necessary
template<typename T>
bool rgbaFromHostAsync(
  const T * ptr_host, const int height,
  const int width, ColorImage * image_out,
  Image<T> * image_tmp,
  const CudaStream & cuda_stream);

/// Convert host/device float -> device depth. This is a simple mem copy
/// Output image is resized if necessary
bool depthFromFloatHostOrDeviceAsync(
  const float * ptr_device, const int height,
  const int width, DepthImage * depth_image,
  const CudaStream & cuda_stream);

/// Convert device int16 -> device depth
/// Output image is resized if necessary
bool depthFromIntDeviceAsync(
  const int16_t * ptr_device, const int height,
  const int width, DepthImage * depth_image,
  const CudaStream & cuda_stream);

/// Convert host int16 -> device depth
/// Output and tmp images are resized if necessary
bool depthFromIntHostAsync(
  const int16_t * ptr_host, const int height,
  const int width, DepthImage * depth_image,
  Image<int16_t> * image_tmp,
  const CudaStream & cuda_stream);


}  // namespace conversions
}  // namespace nvblox

#endif  // NVBLOX_ROS__CONVERSIONS__IMAGE_CONVERSIONS_THRUST_HPP_
