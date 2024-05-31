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

#include "nvblox_ros/camera_cache.hpp"

namespace nvblox
{

namespace
{
bool camerasEqual(const nvblox::Camera & c1, const nvblox::Camera & c2)
{
  bool equal = (c1.fu() == c2.fu()) && (c1.fv() == c2.fv()) && (c1.cu() == c2.cu()) &&
    (c1.cv() == c2.cv()) && (c1.width() == c2.width()) && (c1.height() == c2.height());

  return equal;
}
}  // namespace

void CameraCache::update(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  const Camera camera = conversions::cameraFromMessage(*camera_info_msg);

  // If the camera already exists in the cache, we check that it hasn't changed
  const auto itr = cache_.find(camera_info_msg->header.frame_id);
  if (itr != cache_.end()) {
    CHECK(camerasEqual(itr->second, camera));
  } else {
    cache_[camera_info_msg->header.frame_id] = conversions::cameraFromMessage(*camera_info_msg);
  }
}

bool CameraCache::hasCameraForFrameId(const std::string & frame_id)
{
  return getCameraForFrameId(frame_id).has_value();
}

std::optional<Camera> CameraCache::getCameraForFrameId(const std::string & frame_id)
{
  auto itr = cache_.find(frame_id);
  if (itr != cache_.end()) {
    return itr->second;
  } else {
    return std::nullopt;
  }
}

}  // namespace nvblox
