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

#include <memory>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>

#include <nvblox_msgs/msg/mesh.hpp>

#include "nvblox_rviz_plugin/nvblox_mesh_visual.h"

namespace nvblox_rviz_plugin {

class NvbloxMeshVisual;

class __attribute__((visibility("default"))) NvbloxMeshDisplay
    : public rviz_common::MessageFilterDisplay<nvblox_msgs::msg::Mesh> {
  Q_OBJECT
 public:
  NvbloxMeshDisplay();
  virtual ~NvbloxMeshDisplay();

 public Q_SLOTS:
  virtual void updateCeilingOptions();
 public Q_SLOTS:
  virtual void updateMeshColorOptions();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private:
  void processMessage(
      const nvblox_msgs::msg::Mesh::ConstSharedPtr msg) override;

  rviz_common::properties::BoolProperty* cut_ceiling_property_;
  rviz_common::properties::FloatProperty* ceiling_height_property_;
  rviz_common::properties::EnumProperty* mesh_color_property_;

  std::unique_ptr<NvbloxMeshVisual> visual_;
};

}  // namespace nvblox_rviz_plugin
