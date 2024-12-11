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

#pragma once

#include <QObject>
#include <OgreManualObject.h>
#include <string>
#include <memory>

#include <nvblox_msgs/msg/mesh.hpp>

#include "nvblox_rviz_plugin/nvblox_hash_utils.h"

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include "rviz_common/ros_topic_display.hpp"

namespace nvblox_rviz_plugin
{

class NvbloxVisualParams : public QObject
{
  Q_OBJECT

public:
  explicit NvbloxVisualParams(rviz_common::_RosTopicDisplay * parent);

  float getCeilingHeight();
  bool getCutCeiling();

  bool paramsChanged();

public Q_SLOTS:
  virtual void updateCeilingOptions();

private:
  rviz_common::properties::BoolProperty * cut_ceiling_property_ = nullptr;
  rviz_common::properties::FloatProperty * ceiling_height_property_ = nullptr;

  bool params_changed_ = false;
};


/// Visualizes a single nvblox_msgs::Mesh message.
template<typename MessageType>
class NvbloxBaseVisual
{
public:
  NvbloxBaseVisual(
    rviz_common::_RosTopicDisplay * parent, Ogre::SceneManager * scene_manager,
    Ogre::SceneNode * parent_node);
  virtual ~NvbloxBaseVisual();

  void setMessage(const typename MessageType::ConstSharedPtr & msg);

  /// Set the coordinate frame pose.
  void setFramePosition(const Ogre::Vector3 & position);
  void setFrameOrientation(const Ogre::Quaternion & orientation);

private:
  void initializeFallbackMaterial();
  void initializeBoxMaterial();
  std::unique_ptr<NvbloxVisualParams> params_;

  void maybeCutCeiling(const bool cut_ceiling, const float ceiling_height);

  Ogre::SceneNode * frame_node_ = nullptr;
  Ogre::SceneManager * scene_manager_ = nullptr;
  Ogre::MaterialPtr voxel_material_ = nullptr;

  unsigned int instance_number_{0};
  static unsigned int instance_counter_;

  float block_size_m_ = 0.0f;

  bool using_fallback_voxel_material_ = false;
  nvblox_rviz_plugin::Index3DHashMapType<Ogre::ManualObject *>::type object_map_;
};

using NvbloxMeshVisual = NvbloxBaseVisual<nvblox_msgs::msg::Mesh>;
}  // namespace nvblox_rviz_plugin
