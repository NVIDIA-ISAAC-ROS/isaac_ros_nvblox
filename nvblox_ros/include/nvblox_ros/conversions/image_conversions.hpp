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

#ifndef NVBLOX_ROS__CONVERSIONS__IMAGE_CONVERSIONS_HPP_
#define NVBLOX_ROS__CONVERSIONS__IMAGE_CONVERSIONS_HPP_

#include <nvblox/nvblox.h>

#include <cv_bridge/cv_bridge.h>

#include <string>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rclcpp/rclcpp.hpp>

#include "nvblox_ros/nitros_types.hpp"
#include "nvblox_ros/conversions/image_conversions_thrust.hpp"

namespace nvblox
{
namespace conversions
{
/// Convert camera info message to NVBlox camera object
/// @param camera_info  InputROS message
/// @returns Nvblox camera object
Camera cameraFromMessage(const sensor_msgs::msg::CameraInfo & camera_info);

/// Convert ROS image message to depth frame object
///
/// Output and staging images will be allocated internally if necessary.
///
/// @param image_msg          Input ROS message
/// @param depth_image        Output Nvblox depth image
/// @param image_tmp          Scratch image
/// @param logger             ROS2 logger
/// @param cuda_stream        Cuda stream used for copying data
/// @return True on success, False on failure.
bool depthImageFromRosMessageAsync(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  DepthImage * depth_image, Image<int16_t> * image_tmp, rclcpp::Logger logger,
  const CudaStream & cuda_stream);

/// Convert ROS image message to color image object
/// See @depthImageFromImageMessageAsync for params
bool colorImageFromImageMessageAsync(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  ColorImage * color_image, Image<Rgb> * rgb_image_tmp,
  Image<Bgra> * bgra_image_tmp, rclcpp::Logger logger,
  const CudaStream & cuda_stream);

/// Convert ROS image message to mono image object
/// See @depthImageFromImageMessageAsync for params
bool monoImageFromImageMessageAsync(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  MonoImage * mono_image, const CudaStream & cuda_stream);

/// Convert depth frame to image message
///
/// @param depth_frame  Input nvblox depth frame
/// @param frame_id     ROS frame ID
/// @param image_msg    Resulting image message
/// @param cuda_stream  Cuda stream used for data copy. Will be synchronized
void imageMessageFromDepthImage(
  const DepthImage & depth_frame,
  const std::string & frame_id,
  sensor_msgs::msg::Image * image_msg, const CudaStream & cuda_stream);

/// Convert color frame to image message.
/// See @imageMessageFromDepthImage for params
void imageMessageFromColorImage(
  const ColorImage & color_image,
  const std::string & frame_id,
  sensor_msgs::msg::Image * image_msg, const CudaStream & cuda_stream);


/// Convert Nitros image message to depth frame object
///
/// Output and staging images will be allocated internally if necessary.
///
/// @param view         Input nitros message
/// @param depth_image  Output Nvblox depth image
/// @param logger       ROS2 logger
/// @param cuda_stream  Cuda stream used for copying data
/// @return True on success, False on failure.
bool depthImageFromNitrosViewAsync(
  const NitrosView & view,
  DepthImage * depth_image, rclcpp::Logger logger,
  const CudaStream & cuda_stream);

/// Convert Nitros image message to color frame object
///
/// Output and staging images will be allocated internally if necessary.
///
/// @param view         Input nitros message
/// @param color_image  Output Nvblox color image
/// @param logger       ROS2 logger
/// @param cuda_stream  Cuda stream used for copying data
/// @return True on success, False on failure.
bool colorImageFromNitrosViewAsync(
  const NitrosView & view,
  ColorImage * color_image,
  rclcpp::Logger logger, const CudaStream & cuda_stream);

}  // namespace conversions
}  // namespace nvblox

#endif  // NVBLOX_ROS__CONVERSIONS__IMAGE_CONVERSIONS_HPP_
