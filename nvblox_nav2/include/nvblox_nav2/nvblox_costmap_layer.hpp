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

#ifndef NVBLOX_NAV2__NVBLOX_COSTMAP_LAYER_HPP_
#define NVBLOX_NAV2__NVBLOX_COSTMAP_LAYER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <memory>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nvblox_msgs/msg/distance_map_slice.hpp>

namespace nvblox
{
namespace nav2
{

class NvbloxCostmapLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  NvbloxCostmapLayer();

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x,
    double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i,
    int min_j, int max_i, int max_j) override;

  void reset() override {}
  bool isClearable() override {return true;}

  void sliceCallback(
    const nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice);

private:
  bool lookupInSlice(const Eigen::Vector2f & pos, float * distance);

  // Settings
  bool convert_to_binary_costmap_ = false;
  float max_obstacle_distance_ = 1.0f;
  float inflation_distance_ = 0.5f;
  // This should not include any "special" values like 255.
  uint8_t max_cost_value_ = 252;

  // Subscribers
  rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr
    slice_sub_;

  // State
  nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice_;

  // Global frame of the nav2 costmap.
  // This should match with the global_frame of the corresponding global_costmap/local_costmap.
  // ! changed default value from "odom" to "map" to resolve "Can't transform: odom to map. Error: Lookup would require extrapolation into the future." 
  std::string nav2_costmap_global_frame_ = "map";

  // Listener for slice to global frame transform.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  Eigen::Isometry2f T_G_S_;
};

}  // namespace nav2
}  // namespace nvblox

#endif  // NVBLOX_NAV2__NVBLOX_COSTMAP_LAYER_HPP_
