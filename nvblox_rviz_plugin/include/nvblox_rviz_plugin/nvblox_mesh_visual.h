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

#pragma once

#include <OgreManualObject.h>

#include <nvblox_msgs/msg/mesh.hpp>

#include "nvblox_rviz_plugin/nvblox_hash_utils.h"

namespace nvblox_rviz_plugin {

/// Visualizes a single nvblox_msgs::Mesh message.
class NvbloxMeshVisual {
 public:
  enum MeshColor { kColor = 0, kLambertColor = 1, kNormals = 2 };

  NvbloxMeshVisual(Ogre::SceneManager* scene_manager,
                   Ogre::SceneNode* parent_node);
  virtual ~NvbloxMeshVisual();

  void setMessage(const nvblox_msgs::msg::Mesh::ConstSharedPtr& msg);

  /// Set the coordinate frame pose.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void setCeilingCutoff(bool cut_ceiling, float ceiling_height);
  void setMeshColor(MeshColor mesh_color);

 private:
  // Color helpers.
  Ogre::Vector3 lambertShading(const Ogre::Vector3& normal,
                               const Ogre::Vector3& light,
                               const Ogre::Vector3& color) const;

  std_msgs::msg::ColorRGBA getMeshColorFromColorAndNormal(
      const std_msgs::msg::ColorRGBA& color,
      const geometry_msgs::msg::Point32& normal) const;

  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;

  unsigned int instance_number_;
  static unsigned int instance_counter_;

  bool cut_ceiling_ = false;
  float ceiling_height_ = 0.0f;
  MeshColor mesh_color_ = MeshColor::kColor;

  float block_size_ = 0.0f;

  nvblox_rviz_plugin::Index3DHashMapType<Ogre::ManualObject*>::type object_map_;
};

}  // namespace nvblox_rviz_plugin
