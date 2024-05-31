// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "nvblox_ros/conversions/image_conversions_thrust.hpp"

#include <thrust/device_ptr.h>
#include <thrust/transform.h>

namespace nvblox {
namespace conversions {

/// Resizes an image if necessary
template <class ImageType>
void maybeReallocateImage(ImageType* image, const int height, const int width,
                          MemoryType memory_type) {
  CHECK(image != nullptr);
  if (image->width() != width || image->height() != height ||
      image->memory_type() != memory_type) {
    *image = ImageType(height, width, memory_type);
  }
}

// Conversion from int16 to float depth image by dividing with 1000, according
// to REP-118 (https://ros.org/reps/rep-0118.html)
struct DivideBy1000 : public thrust::unary_function<uint16_t, float> {
  static constexpr float kInv1000 = 1.0f / 1000.0f;

  __host__ __device__ float operator()(const uint16_t& in) {
    return static_cast<float>(in) * kInv1000;
  }
};

template <typename T>
struct ToRgba : public thrust::unary_function<T, Rgba> {
  __host__ __device__ Rgba operator()(const T& in);
};

// Conversion from RGB to RGBA image where alpha is set to 255
template <>
struct ToRgba<Rgb> : public thrust::unary_function<Rgb, Rgba> {
  __host__ __device__ Rgba operator()(const Rgb& in) {
    return Rgba(in[0], in[1], in[2], 255);
  }
};
// Conversion from BGRA to RGBA image
template <>
struct ToRgba<Bgra> : public thrust::unary_function<Bgra, Rgba> {
  __host__ __device__ Rgba operator()(const Bgra& in) {
    return Rgba(in[2], in[1], in[0], in[4]);
  }
};

template <typename T>
bool rgbaFromDeviceAsync(const T* ptr_device, const int height, const int width,
                         ColorImage* color_image,
                         const CudaStream& cuda_stream) {
  CHECK_NOTNULL(ptr_device);
  CHECK_NOTNULL(color_image);
  CHECK(height > 0);
  CHECK(width > 0);
  maybeReallocateImage(color_image, height, width, MemoryType::kDevice);

  thrust::device_ptr<const T> thrust_ptr(ptr_device);

  thrust::transform(thrust::cuda::par.on(cuda_stream), thrust_ptr,
                    thrust_ptr + width * height, color_image->dataPtr(),
                    ToRgba<T>());

  return true;
}

template bool rgbaFromDeviceAsync<Rgb>(const Rgb*, const int, const int,
                                       ColorImage*, const CudaStream&);
template bool rgbaFromDeviceAsync<Bgra>(const Bgra*, const int, const int,
                                        ColorImage*, const CudaStream&);

template <typename T>
bool rgbaFromHostAsync(const T* ptr_host, const int height, const int width,
                       ColorImage* image_out, Image<T>* image_tmp,
                       const CudaStream& cuda_stream) {
  CHECK_NOTNULL(ptr_host);
  CHECK_NOTNULL(image_out);
  CHECK_NOTNULL(image_tmp);
  CHECK(height > 0);
  CHECK(width > 0);

  maybeReallocateImage(image_tmp, height, width, MemoryType::kDevice);
  image_tmp->copyFromAsync(height, width, ptr_host, cuda_stream);

  return rgbaFromDeviceAsync(image_tmp->dataPtr(), height, width, image_out,
                             cuda_stream);
}

template bool rgbaFromHostAsync<Rgb>(const Rgb*, const int, const int,
                                     ColorImage*, Image<Rgb>*,
                                     const CudaStream&);
template bool rgbaFromHostAsync<Bgra>(const Bgra*, const int, const int,
                                      ColorImage*, Image<Bgra>*,
                                      const CudaStream&);

bool depthFromFloatHostOrDeviceAsync(const float* ptr_device, const int height,
                                     const int width, DepthImage* image_out,
                                     const CudaStream& cuda_stream) {
  CHECK_NOTNULL(ptr_device);
  CHECK_NOTNULL(image_out);
  CHECK(height > 0);
  CHECK(width > 0);
  maybeReallocateImage(image_out, height, width, MemoryType::kDevice);

  image_out->copyFromAsync(height, width, ptr_device, cuda_stream);

  return true;
}

bool depthFromIntDeviceAsync(const int16_t* ptr_device, const int height,
                             const int width, DepthImage* image_out,
                             const CudaStream& cuda_stream) {
  CHECK_NOTNULL(ptr_device);
  CHECK_NOTNULL(image_out);
  CHECK(height > 0);
  CHECK(width > 0);
  maybeReallocateImage(image_out, height, width, MemoryType::kDevice);

  thrust::device_ptr<const int16_t> thrust_ptr(ptr_device);

  thrust::transform(thrust::cuda::par.on(cuda_stream), thrust_ptr,
                    thrust_ptr + width * height, image_out->dataPtr(),
                    DivideBy1000());

  return true;
}

bool depthFromIntHostAsync(const int16_t* ptr_host, const int height,
                           const int width, DepthImage* image_out,
                           Image<int16_t>* image_tmp,
                           const CudaStream& cuda_stream) {
  CHECK_NOTNULL(ptr_host);
  CHECK_NOTNULL(image_out);
  CHECK_NOTNULL(image_tmp);
  CHECK(height > 0);
  CHECK(width > 0);

  maybeReallocateImage(image_tmp, height, width, MemoryType::kDevice);
  image_tmp->copyFromAsync(height, width, ptr_host, cuda_stream);

  return depthFromIntDeviceAsync(image_tmp->dataPtr(), height, width, image_out,
                                 cuda_stream);
}

}  // namespace conversions

}  // namespace nvblox