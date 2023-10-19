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

#include "nvblox_nav2/nvblox_costmap_layer.hpp"

#include <string>

#include <nav2_costmap_2d/costmap_math.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <rclcpp/parameter_events_filter.hpp>

namespace nvblox
{
namespace nav2
{

NvbloxCostmapLayer::NvbloxCostmapLayer() {}

void NvbloxCostmapLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  enabled_ = node->declare_parameter(name_ + "." + "enabled", true);

  // Get the path of the map slice topic.
  std::string nvblox_map_slice_topic = "/nvblox_node/static_map_slice";

  nvblox_map_slice_topic = node->declare_parameter<std::string>(
    getFullName("nvblox_map_slice_topic"), nvblox_map_slice_topic);
  max_obstacle_distance_ = node->declare_parameter<float>(
    getFullName("max_obstacle_distance"), max_obstacle_distance_);
  inflation_distance_ = node->declare_parameter<float>(
    getFullName("inflation_distance"), inflation_distance_);
  max_cost_value_ = node->declare_parameter<uint8_t>(
    getFullName("max_cost_value"), max_cost_value_);

  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "Name: " << name_ << " Topic name: " << nvblox_map_slice_topic <<
      " Max obstacle distance: " << max_obstacle_distance_);

  // Add subscribers to the nvblox message.
  slice_sub_ = node->create_subscription<nvblox_msgs::msg::DistanceMapSlice>(
    nvblox_map_slice_topic, 1,
    std::bind(
      &NvbloxCostmapLayer::sliceCallback, this,
      std::placeholders::_1));
  current_ = true;
}

// The method is called to ask the plugin: which area of costmap it needs to
// update.
void NvbloxCostmapLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/,
  double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x,
  double * max_y)
{
  // We give the full AABB of the map that we have available to the
  // upstream controller.
  // According to nav2 the bounds can only grow bigger. So we only update the
  // bounds if it grows bigger or we keep the old values
  if (slice_ != nullptr) {
    const double current_plugin_min_x = slice_->origin.x;
    const double current_plugin_min_y = slice_->origin.y;
    const double current_plugin_max_x = current_plugin_min_x + slice_->width * slice_->resolution;
    const double current_plugin_max_y = current_plugin_min_y + slice_->height * slice_->resolution;

    if (current_plugin_min_x < *min_x) {
      *min_x = current_plugin_min_x;
    }
    if (current_plugin_max_x > *max_x) {
      *max_x = current_plugin_max_x;
    }
    if (current_plugin_min_y < *min_y) {
      *min_y = current_plugin_min_y;
    }
    if (current_plugin_max_y > *max_y) {
      *max_y = current_plugin_max_y;
    }
  }

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  RCLCPP_DEBUG(
    node->get_logger(),
    "Update bounds: Min x: %f Min y: %f Max x: %f Max y: %f", *min_x,
    *min_y, *max_x, *max_y);
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous
// layers.
void NvbloxCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  RCLCPP_DEBUG(
    node->get_logger(),
    "Update costs: Min i: %d Min j: %d Max i: %d Max j: %d", min_i,
    min_j, max_i, max_j);
  // Copy over the relevant values to the internal costmap.
  // We need to convert from meters to cell units.
  // We'll just directly write to the costmap layer.
  setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
  matchSize();
  uint8_t * costmap_array = getCharMap();
  unsigned int size_x = getSizeInCellsX(), size_y = getSizeInCellsY();

  RCLCPP_DEBUG(
    node->get_logger(), "Size in cells x: %d size in cells y: %d",
    size_x, size_y);

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = getIndex(i, j);

      // Figure out the world coordinates of this gridcell.
      double world_x, world_y;
      mapToWorld(i, j, world_x, world_y);

      // Look up the corresponding cell in our latest map.
      float distance = 0.0f;
      bool valid = lookupInSlice(Eigen::Vector2f(world_x, world_y), &distance);

      // Calculate what this maps to in the original structure.
      uint8_t cost = nav2_costmap_2d::NO_INFORMATION;
      if (valid) {
        // Convert the distance value to a costmap value.
        if (distance <= 0.0f) {
          cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        } else if (distance < inflation_distance_) {
          cost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        } else if (distance > max_obstacle_distance_) {
          cost = nav2_costmap_2d::FREE_SPACE;
        } else {
          cost = static_cast<uint8_t>(
            max_cost_value_ *
            (1.0f - std::min<float>(
              (distance - inflation_distance_) /
              max_obstacle_distance_,
              1.0f)));
        }
      }
      costmap_array[index] = cost;
    }
  }

  // This combines the master costmap with the current costmap by taking
  // the max across all costmaps.
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  RCLCPP_DEBUG(node->get_logger(), "Finished updating.");
  current_ = true;
}

void NvbloxCostmapLayer::sliceCallback(
  const nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  RCLCPP_DEBUG(node->get_logger(), "Slice callback.");
  slice_ = slice;
}

bool NvbloxCostmapLayer::lookupInSlice(
  const Eigen::Vector2f & pos,
  float * distance)
{
  // TODO(helen): if coordinate systems don't match, look up in TF.
  // This would allow our map to also be used for global planning.
  // If we don't have any slice, we don't have any costs. :(
  if (slice_ == nullptr) {
    return false;
  }

  // Else look up in the actual slice.
  Eigen::Vector2f scaled_position =
    (pos - Eigen::Vector2f(slice_->origin.x, slice_->origin.y)) /
    slice_->resolution;
  Eigen::Vector2i pos_index = scaled_position.array().round().cast<int>();

  if (pos_index.x() < 0 || pos_index.x() >= static_cast<int>(slice_->width) ||
    pos_index.y() < 0 || pos_index.y() >= static_cast<int>(slice_->height))
  {
    return false;
  }

  size_t linear_index = pos_index.y() * slice_->width + pos_index.x();

  if (linear_index >= slice_->width * slice_->height) {
    return false;
  }

  *distance = slice_->data[linear_index];
  // The value can still be allocated but unknown. Handle this case too.
  if (*distance != slice_->unknown_value) {
    return true;
  }
  return false;
}

}  // namespace nav2
}  // namespace nvblox

// Register the macro for this layer
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nvblox::nav2::NvbloxCostmapLayer, nav2_costmap_2d::Layer)
