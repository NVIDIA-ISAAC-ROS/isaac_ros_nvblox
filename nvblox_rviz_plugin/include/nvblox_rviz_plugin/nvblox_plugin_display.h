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

#include <memory>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>

#include <nvblox_msgs/msg/mesh.hpp>
#include <nvblox_msgs/msg/voxel_block_layer.hpp>

#include "nvblox_rviz_plugin/nvblox_plugin_visual.h"

namespace nvblox_rviz_plugin
{

template<typename MessageType>
class __attribute__((visibility("default"))) NvbloxBaseDisplay : public rviz_common::
  MessageFilterDisplay<MessageType>
{
public:
  NvbloxBaseDisplay() = default;
  virtual ~NvbloxBaseDisplay();

protected:
  virtual void onInitialize();
  virtual void reset();
  void updatePose(const typename MessageType::ConstSharedPtr msg);

private:
  void processMessage(const typename MessageType::ConstSharedPtr msg) override;
  void maybeInitialize();
  std::unique_ptr<NvbloxBaseVisual<MessageType>> visual_;
};

using NvbloxMeshDisplay = NvbloxBaseDisplay<nvblox_msgs::msg::Mesh>;
using NvbloxVoxelBlockLayerDisplay = NvbloxBaseDisplay<nvblox_msgs::msg::VoxelBlockLayer>;

}  // namespace nvblox_rviz_plugin
