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
