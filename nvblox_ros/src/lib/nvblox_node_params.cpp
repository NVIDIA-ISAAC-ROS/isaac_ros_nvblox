// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "nvblox_ros/nvblox_node_params.hpp"
#include "nvblox_ros/mapper_initialization.hpp"

namespace nvblox
{

// Declare a ROS parameter and assign its value to param
template<typename ParamType, typename RosParamType = ParamType>
void initParam(
  rclcpp::Node * node, Param<ParamType> * param,
  std::optional<nvblox::parameters::ParameterTreeNode *> tree = std::nullopt)
{
  const auto desc = param->description();
  rcl_interfaces::msg::ParameterDescriptor ros_desc;
  ros_desc.description = desc.help_string;

  *param = static_cast<ParamType>(node->declare_parameter<RosParamType>(
      desc.name, static_cast<RosParamType>(desc.default_value), ros_desc));

  // Add to the tree, if requested
  if (tree.has_value()) {
    CHECK(tree.value()->children().has_value())
      << "Declare parameter requires an initialized non-leaf ParameterTreeNode.";
    tree.value()->children().value().push_back(
      nvblox::parameters::ParameterTreeNode(desc.name, static_cast<ParamType>(*param)));
  }
}

// Declare a ROS string parameter and assign its value to param
void initStringParam(
  rclcpp::Node * node, StringParam * param,
  std::optional<nvblox::parameters::ParameterTreeNode *> tree = std::nullopt)
{
  const auto desc = param->description();
  rcl_interfaces::msg::ParameterDescriptor ros_desc;
  ros_desc.description = desc.help_string;

  *param = node->declare_parameter<std::string>(desc.name, desc.default_value, ros_desc);

  // Add to the tree, if requested
  if (tree.has_value()) {
    CHECK(tree.value()->children().has_value())
      << "Declare parameter requires an initialized non-leaf ParameterTreeNode.";
    tree.value()->children().value().push_back(
      nvblox::parameters::ParameterTreeNode(desc.name, std::string(*param)));
  }
}

void initializeRosParams(
  rclcpp::Node * node, NvbloxNodeParams * params,
  std::optional<nvblox::parameters::ParameterTreeNode *> parameter_tree)
{
  initStringParam(node, &params->global_frame, parameter_tree);
  initStringParam(node, &params->pose_frame, parameter_tree);
  initStringParam(node, &params->map_clearing_frame_id, parameter_tree);
  initStringParam(node, &params->slice_visualization_attachment_frame_id, parameter_tree);

  initParam<bool>(node, &params->publish_esdf_distance_slice, parameter_tree);
  initParam<bool>(node, &params->use_color, parameter_tree);
  initParam<bool>(node, &params->use_depth, parameter_tree);
  initParam<bool>(node, &params->use_lidar, parameter_tree);
  initParam<bool>(node, &params->use_non_equal_vertical_fov_lidar_params, parameter_tree);
  initParam<bool>(node, &params->print_timings_to_console, parameter_tree);
  initParam<bool>(node, &params->print_rates_to_console, parameter_tree);
  initParam<bool>(node, &params->print_delays_to_console, parameter_tree);
  initParam<bool>(node, &params->enable_mesh_markers, parameter_tree);
  initParam<bool>(node, &params->layer_visualization_undo_gamma_correction, parameter_tree);

  initParam<int>(node, &params->num_cameras, parameter_tree);
  initParam<int>(node, &params->lidar_width, parameter_tree);
  initParam<int>(node, &params->lidar_height, parameter_tree);
  initParam<int>(node, &params->tick_period_ms, parameter_tree);
  initParam<int>(node, &params->print_statistics_on_console_period_ms, parameter_tree);
  initParam<int>(node, &params->maximum_sensor_message_queue_length, parameter_tree);
  initParam<int>(node, &params->back_projection_subsampling, parameter_tree);

  initParam<float>(node, &params->lidar_vertical_fov_rad, parameter_tree);
  initParam<float>(node, &params->voxel_size, parameter_tree);
  initParam<float>(node, &params->min_angle_below_zero_elevation_rad, parameter_tree);
  initParam<float>(node, &params->max_angle_above_zero_elevation_rad, parameter_tree);
  initParam<float>(node, &params->slice_visualization_side_length, parameter_tree);
  initParam<float>(node, &params->layer_visualization_max_tsdf_distance_m, parameter_tree);
  initParam<float>(node, &params->layer_visualization_min_tsdf_weight, parameter_tree);
  initParam<float>(node, &params->layer_visualization_exclusion_height_m, parameter_tree);
  initParam<float>(node, &params->layer_visualization_exclusion_radius_m, parameter_tree);
  initParam<float>(node, &params->integrate_depth_rate_hz, parameter_tree);
  initParam<float>(node, &params->integrate_color_rate_hz, parameter_tree);
  initParam<float>(node, &params->integrate_lidar_rate_hz, parameter_tree);
  initParam<float>(node, &params->update_mesh_rate_hz, parameter_tree);
  initParam<float>(node, &params->update_esdf_rate_hz, parameter_tree);
  initParam<float>(node, &params->publish_layer_rate_hz, parameter_tree);
  initParam<float>(node, &params->decay_tsdf_rate_hz, parameter_tree);
  initParam<float>(node, &params->decay_dynamic_occupancy_rate_hz, parameter_tree);
  initParam<float>(node, &params->clear_map_outside_radius_rate_hz, parameter_tree);
  initParam<float>(node, &params->map_clearing_radius_m, parameter_tree);
  initParam<float>(node, &params->max_back_projection_distance, parameter_tree);
  initParam<float>(node, &params->esdf_and_gradients_unobserved_value, parameter_tree);

  initParam<EsdfMode, int>(node, &params->esdf_mode, parameter_tree);

  initStringParam(node, &params->mapping_type_str, parameter_tree);
  params->mapping_type = mapping_type_from_string(params->mapping_type_str, node);
}

}  // namespace nvblox
