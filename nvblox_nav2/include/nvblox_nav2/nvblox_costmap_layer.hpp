/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef NVBLOX_NAV2__NVBLOX_COSTMAP_LAYER_HPP_
#define NVBLOX_NAV2__NVBLOX_COSTMAP_LAYER_HPP_

#include <Eigen/Core>
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
  float max_obstacle_distance_ = 1.0f;
  float inflation_distance_ = 0.5f;
  // This should not include any "special" values like 255.
  uint8_t max_cost_value_ = 252;

  // Subscribers
  rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr
    slice_sub_;

  // State
  nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice_;
};

}  // namespace nav2
}  // namespace nvblox

#endif  // NVBLOX_NAV2__NVBLOX_COSTMAP_LAYER_HPP_
