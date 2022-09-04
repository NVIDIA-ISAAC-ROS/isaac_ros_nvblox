/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/visualization_manager.hpp>

#include "nvblox_rviz_plugin/nvblox_mesh_display.h"

namespace nvblox_rviz_plugin {

NvbloxMeshDisplay::NvbloxMeshDisplay() {
  cut_ceiling_property_ = new rviz_common::properties::BoolProperty(
      "Cut Ceiling", false,
      "If set to true, will not visualize anything above a certain z value.",
      this, SLOT(updateCeilingOptions()));

  ceiling_height_property_ = new rviz_common::properties::FloatProperty(
      "Ceiling Height", 0.0,
      "Height above which the visualization will be cut off.", this,
      SLOT(updateCeilingOptions()));

  mesh_color_property_ = new rviz_common::properties::EnumProperty(
      "Mesh Color", "Color + Shading", "How to color the displayed mesh.", this,
      SLOT(updateMeshColorOptions()));

  // Set up valid options.
  mesh_color_property_->addOption("Color", NvbloxMeshVisual::MeshColor::kColor);
  mesh_color_property_->addOption("Color + Shading",
                                  NvbloxMeshVisual::MeshColor::kLambertColor);
  mesh_color_property_->addOption("Normals",
                                  NvbloxMeshVisual::MeshColor::kNormals);
}

void NvbloxMeshDisplay::updateCeilingOptions() {
  if (visual_ != nullptr) {
    visual_->setCeilingCutoff(cut_ceiling_property_->getBool(),
                              ceiling_height_property_->getFloat());
  }
}

void NvbloxMeshDisplay::updateMeshColorOptions() {
  if (visual_ != nullptr) {
    visual_->setMeshColor(static_cast<NvbloxMeshVisual::MeshColor>(
        mesh_color_property_->getOptionInt()));
  }
}

void NvbloxMeshDisplay::onInitialize() { MFDClass::onInitialize(); }

NvbloxMeshDisplay::~NvbloxMeshDisplay() {}

void NvbloxMeshDisplay::reset() {
  MFDClass::reset();
  visual_.reset();
}

void NvbloxMeshDisplay::processMessage(
    const nvblox_msgs::msg::Mesh::ConstSharedPtr msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    RCLCPP_ERROR(rclcpp::get_logger("nvblox"),
                 "Error transforming from frame '%s' to frame '%s'",
                 msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  if (visual_ == nullptr) {
    visual_.reset(
        new NvbloxMeshVisual(context_->getSceneManager(), scene_node_));
    visual_->setCeilingCutoff(cut_ceiling_property_->getBool(),
                              ceiling_height_property_->getFloat());
    visual_->setMeshColor(static_cast<NvbloxMeshVisual::MeshColor>(
        mesh_color_property_->getOptionInt()));
  }

  // Now set or update the contents of the chosen visual.
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

}  // namespace nvblox_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nvblox_rviz_plugin::NvbloxMeshDisplay,
                       rviz_common::Display)
