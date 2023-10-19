// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

namespace nvblox
{
namespace conversions
{

// Convert camera info message to NVBlox camera object
Camera cameraFromMessage(const sensor_msgs::msg::CameraInfo & camera_info);

// Convert image to depth frame object
bool depthImageFromImageMessage(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  DepthImage * depth_frame);

bool colorImageFromImageMessage(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  ColorImage * color_image);

bool monoImageFromImageMessage(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  MonoImage * mono_image);

// Convert depth frame to image message.
void imageMessageFromDepthImage(
  const DepthImage & depth_frame,
  const std::string & frame_id,
  sensor_msgs::msg::Image * image_msg);

// Convert color frame to image message.
void imageMessageFromColorImage(
  const ColorImage & color_image,
  const std::string & frame_id,
  sensor_msgs::msg::Image * image_msg);

}  // namespace conversions
}  // namespace nvblox

#endif  // NVBLOX_ROS__CONVERSIONS__IMAGE_CONVERSIONS_HPP_
