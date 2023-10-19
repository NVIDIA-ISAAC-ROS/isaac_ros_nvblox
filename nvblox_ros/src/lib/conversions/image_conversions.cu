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

#include <thrust/transform.h>

#include "nvblox_ros/conversions/image_conversions.hpp"

namespace nvblox {
namespace conversions {

// TODO(remosteiner): Group these functions into a class with a 
// cuda stream member and call copyFromAsync instead of copyFrom.

// Convert camera info message to NVBlox camera object
Camera cameraFromMessage(
    const sensor_msgs::msg::CameraInfo& camera_info) {
  Camera camera(camera_info.k[0], camera_info.k[4], camera_info.k[2],
                camera_info.k[5], camera_info.width, camera_info.height);
  return camera;
}

// Convert image to GPU image
bool colorImageFromImageMessage(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    ColorImage* color_image) {
  CHECK_NOTNULL(color_image);

  // Convert to our color encoding
  cv_bridge::CvImageConstPtr rgba_cv_image =
      cv_bridge::toCvCopy(image_msg, "rgba8");

  color_image->copyFrom(
      image_msg->height, image_msg->width,
      reinterpret_cast<const Color*>(rgba_cv_image->image.ptr()));

  return true;
}

// Convert image to GPU image
bool monoImageFromImageMessage(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    MonoImage* mono_image) {
  CHECK_NOTNULL(mono_image);

  // First check if we actually have a valid image here.
  if (image_msg->encoding != "mono8") {
    return false;
  }

  mono_image->copyFrom(image_msg->height, image_msg->width,
                       &image_msg->data[0]);
  return true;
}

void imageMessageFromDepthImage(
    const DepthImage& depth_image, const std::string& frame_id,
    sensor_msgs::msg::Image* image_msg) {
  CHECK_NOTNULL(image_msg);
  size_t image_size =
      depth_image.width() * depth_image.height() * sizeof(float);
  image_msg->data.resize(image_size);

  image_msg->header.frame_id = frame_id;
  image_msg->width = depth_image.width();
  image_msg->height = depth_image.height();
  image_msg->step = depth_image.width() * sizeof(float);

  image_msg->encoding = "32FC1";

  cudaMemcpy(&image_msg->data[0], depth_image.dataConstPtr(), image_size,
             cudaMemcpyDefault);
}

void imageMessageFromColorImage(
    const ColorImage& color_image, const std::string& frame_id,
    sensor_msgs::msg::Image* image_msg) {
  CHECK_NOTNULL(image_msg);
  constexpr int num_channels = 4;
  size_t image_size = color_image.width() * color_image.height() *
                      sizeof(uint8_t) * num_channels;
  image_msg->data.resize(image_size);

  image_msg->header.frame_id = frame_id;
  image_msg->width = color_image.width();
  image_msg->height = color_image.height();
  image_msg->step = color_image.width() * sizeof(uint8_t) * num_channels;

  image_msg->encoding = "rgba8";

  cudaMemcpy(&image_msg->data[0], color_image.dataConstPtr(), image_size,
             cudaMemcpyDefault);
}

struct DivideBy1000 : public thrust::unary_function<uint16_t, float> {
  __host__ __device__ float operator()(const uint16_t& in) {
    return static_cast<float>(in) / 1000.0f;
  }
};

// Convert image to depth frame object
bool depthImageFromImageMessage(
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
    depth_image->copyFrom(image_msg->height, image_msg->width,
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
      depth_image->copyFrom(image_msg->height, image_msg->width,
                            float_depth_buffer.data());
    }
  }

  return true;
}

} // namespace conversions
}  // namespace nvblox
