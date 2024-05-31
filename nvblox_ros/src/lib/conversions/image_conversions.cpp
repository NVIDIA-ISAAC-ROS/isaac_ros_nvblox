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

#include "nvblox_ros/conversions/image_conversions.hpp"
#include "nvblox_ros/conversions/image_conversions_thrust.hpp"

namespace nvblox
{
namespace conversions
{

// Convert camera info message to NVBlox camera object
Camera cameraFromMessage(const sensor_msgs::msg::CameraInfo & camera_info)
{
  Camera camera(camera_info.k[0], camera_info.k[4], camera_info.k[2], camera_info.k[5],
    camera_info.width, camera_info.height);
  return camera;
}

// Convert image to GPU image
bool colorImageFromImageMessageAsync(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  ColorImage * color_image, Image<Rgb> * rgb_image_tmp,
  Image<Bgra> * bgra_image_tmp, rclcpp::Logger logger,
  const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(color_image);

  const std::string encoding = image_msg->encoding;
  if (encoding == "rgb8") {
    return rgbaFromHostAsync(
      reinterpret_cast<const Rgb *>(&image_msg->data[0]), image_msg->height,
      image_msg->width, color_image, rgb_image_tmp, cuda_stream);
  } else if (encoding == "bgra8") {
    return rgbaFromHostAsync(
      reinterpret_cast<const Bgra *>(&image_msg->data[0]), image_msg->height,
      image_msg->width, color_image, bgra_image_tmp, cuda_stream);
  } else {
    RCLCPP_ERROR_STREAM(logger, "Invalid color image encoding: " << encoding);
    return false;
  }
}

// Convert image to GPU image
bool monoImageFromImageMessageAsync(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  MonoImage * mono_image, const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(mono_image);

  // First check if we actually have a valid image here.
  if (image_msg->encoding != "mono8") {
    return false;
  }

  mono_image->copyFromAsync(image_msg->height, image_msg->width, &image_msg->data[0], cuda_stream);
  return true;
}

void imageMessageFromDepthImage(
  const DepthImage & depth_image, const std::string & frame_id,
  sensor_msgs::msg::Image * image_msg, const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(image_msg);
  size_t image_size = depth_image.width() * depth_image.height() * sizeof(float);
  image_msg->data.resize(image_size);

  image_msg->header.frame_id = frame_id;
  image_msg->width = depth_image.width();
  image_msg->height = depth_image.height();
  image_msg->step = depth_image.width() * sizeof(float);

  image_msg->encoding = "32FC1";

  cudaMemcpyAsync(
    &image_msg->data[0], depth_image.dataConstPtr(), image_size, cudaMemcpyDefault,
    cuda_stream);
  cuda_stream.synchronize();
}

void imageMessageFromColorImage(
  const ColorImage & color_image, const std::string & frame_id,
  sensor_msgs::msg::Image * image_msg, const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(image_msg);
  constexpr int num_channels = 4;
  size_t image_size = color_image.width() * color_image.height() * sizeof(uint8_t) * num_channels;
  image_msg->data.resize(image_size);

  image_msg->header.frame_id = frame_id;
  image_msg->width = color_image.width();
  image_msg->height = color_image.height();
  image_msg->step = color_image.width() * sizeof(uint8_t) * num_channels;

  image_msg->encoding = "rgba8";

  cudaMemcpyAsync(
    &image_msg->data[0], color_image.dataConstPtr(), image_size, cudaMemcpyDefault,
    cuda_stream);
  cuda_stream.synchronize();
}

bool depthImageFromRosMessageAsync(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  DepthImage * depth_image, Image<int16_t> * image_tmp,
  rclcpp::Logger logger, const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(depth_image);

  if (image_msg->encoding == "32FC1") {
    return depthFromFloatHostOrDeviceAsync(
      reinterpret_cast<const float *>(&image_msg->data[0]),
      image_msg->height, image_msg->width, depth_image,
      cuda_stream);
  } else if (image_msg->encoding == "16UC1") {
    return depthFromIntHostAsync(
      reinterpret_cast<const int16_t *>(&image_msg->data[0]),
      image_msg->height, image_msg->width, depth_image, image_tmp,
      cuda_stream);
  } else {
    RCLCPP_ERROR_STREAM(logger, "Invalid depth image encoding: " << image_msg->encoding);
    return false;
  }
}

bool depthImageFromNitrosViewAsync(
  const NitrosView & view, DepthImage * depth_image,
  rclcpp::Logger logger, const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(depth_image);

  if (view.GetEncoding() == "32FC1") {
    return depthFromFloatHostOrDeviceAsync(
      reinterpret_cast<const float *>(view.GetGpuData()),
      view.GetHeight(), view.GetWidth(), depth_image,
      cuda_stream);
  } else if (view.GetEncoding() == "16UC1" || view.GetEncoding() == "mono16") {
    return depthFromIntDeviceAsync(
      reinterpret_cast<const int16_t *>(view.GetGpuData()),
      view.GetHeight(), view.GetWidth(), depth_image, cuda_stream);
  } else {
    RCLCPP_ERROR_STREAM(logger, "Invalid depth image encoding: " << view.GetEncoding());
    return false;
  }
}

bool colorImageFromNitrosViewAsync(
  const NitrosView & view, ColorImage * color_image,
  rclcpp::Logger logger, const CudaStream & cuda_stream)
{
  CHECK_NOTNULL(color_image);

  const std::string encoding = view.GetEncoding();
  if (encoding == "rgb8") {
    return rgbaFromDeviceAsync(
      reinterpret_cast<const Rgb *>(view.GetGpuData()), view.GetHeight(),
      view.GetWidth(), color_image, cuda_stream);
  } else if (encoding == "bgra8") {
    return rgbaFromDeviceAsync(
      reinterpret_cast<const Bgra *>(view.GetGpuData()), view.GetHeight(),
      view.GetWidth(), color_image, cuda_stream);
  } else {
    RCLCPP_ERROR_STREAM(logger, "Invalid color image encoding: " << encoding);
    return false;
  }
}

}  // namespace conversions
}  // namespace nvblox
