/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
 
#pragma once

#include <OgreManualObject.h>

#include <nvblox_msgs/msg/mesh.hpp>

#include "nvblox_rviz_plugin/nvblox_hash_utils.h"

namespace nvblox_rviz_plugin {

/// Visualizes a single nvblox_msgs::Mesh message.
class NvbloxMeshVisual {
 public:
  NvbloxMeshVisual(Ogre::SceneManager* scene_manager,
                   Ogre::SceneNode* parent_node);
  virtual ~NvbloxMeshVisual();

  void setMessage(const nvblox_msgs::msg::Mesh::ConstSharedPtr& msg);

  /// Set the coordinate frame pose.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void setCeilingCutoff(bool cut_ceiling, float ceiling_height);

 private:
  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;

  unsigned int instance_number_;
  static unsigned int instance_counter_;

  bool cut_ceiling_ = false;
  float ceiling_height_ = 0.0f;

  float block_size_ = 0.0f;

  nvblox_rviz_plugin::Index3DHashMapType<Ogre::ManualObject*>::type object_map_;
};

}  // namespace nvblox_rviz_plugin
