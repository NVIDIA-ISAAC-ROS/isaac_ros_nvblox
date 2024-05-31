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

#ifndef NVBLOX_ROS__CAMERA_CACHE_HPP_
#define NVBLOX_ROS__CAMERA_CACHE_HPP_

#include <string>

#include <unordered_map>

#include <sensor_msgs/msg/camera_info.hpp>
#include "nvblox/sensors/camera.h"

#include "nvblox_ros/conversions/image_conversions.hpp"

namespace nvblox
{
/// Class for storing cameras intrinsics.
class CameraCache
{
public:
  /// Convert CameraInfo message -> nvblox::camera and update the cache
  void update(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);

  /// Return true if the given frame_id exists in the cache
  bool hasCameraForFrameId(const std::string & frame_id);

  /// Get the camera given a frame_id. nullopt is return if frame_id is not in the cache
  std::optional<Camera> getCameraForFrameId(const std::string & frame_id);

private:
  std::unordered_map<std::string, nvblox::Camera> cache_;
};

}  // namespace nvblox

#endif  // NVBLOX_ROS__CAMERA_CACHE_HPP_
