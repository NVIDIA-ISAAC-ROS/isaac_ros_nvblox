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

#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/visualization_manager.hpp>

#include "nvblox_rviz_plugin/nvblox_plugin_display.h"

namespace nvblox_rviz_plugin
{


template<typename MessageType>
void NvbloxBaseDisplay<MessageType>::onInitialize()
{
  rviz_common::MessageFilterDisplay<MessageType>::MFDClass::onInitialize();
}

template<typename MessageType>
NvbloxBaseDisplay<MessageType>::~NvbloxBaseDisplay() {}

template<typename MessageType>
void NvbloxBaseDisplay<MessageType>::reset()
{
  rviz_common::MessageFilterDisplay<MessageType>::MFDClass::reset();
  visual_.reset();
}

template<typename MessageType>
void NvbloxBaseDisplay<MessageType>::updatePose(const typename MessageType::ConstSharedPtr msg)
{
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!this->context_->getFrameManager()->getTransform(
      msg->header.frame_id, msg->header.stamp, position, orientation))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("nvblox"), "Error transforming from frame '%s' to frame '%s'",
      msg->header.frame_id.c_str(), qPrintable(this->fixed_frame_));
    return;
  }

  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

template<typename MessageType>
void NvbloxBaseDisplay<MessageType>::maybeInitialize()
{
  if (visual_ == nullptr) {
    visual_.reset(
      new NvbloxBaseVisual<MessageType>(
        this,
        this->context_->getSceneManager(),
        this->scene_node_));
  }
}

template<typename MessageType>
void NvbloxBaseDisplay<MessageType>::processMessage(const typename MessageType::ConstSharedPtr msg)
{
  maybeInitialize();
  updatePose(msg);
  visual_->setMessage(msg);
}

template class NvbloxBaseDisplay<nvblox_msgs::msg::VoxelBlockLayer>;
template class NvbloxBaseDisplay<nvblox_msgs::msg::Mesh>;

}  // namespace nvblox_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nvblox_rviz_plugin::NvbloxMeshDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(nvblox_rviz_plugin::NvbloxVoxelBlockLayerDisplay, rviz_common::Display)
