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

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "nvblox_ros/nvblox_node_params.hpp"

namespace nvblox
{

template<typename T> void testParam(rclcpp::Node * node, Param<T> & param)
{
  T from_ros;
  const bool is_set = node->get_parameter<T>(param.description().name, from_ros);
  EXPECT_TRUE(is_set);
  EXPECT_EQ(from_ros, param.get());
}

void testStringParam(rclcpp::Node * node, StringParam & param)
{
  std::string from_ros;
  const bool is_set = node->get_parameter<std::string>(param.description().name, from_ros);
  EXPECT_TRUE(is_set);
  EXPECT_EQ(from_ros, param.get());
}

TEST(NvbloxNodeParams, initialize) {
  constexpr size_t kExpectedParamSize = 1536;
  if (sizeof(NvbloxNodeParams) != kExpectedParamSize) {
    std::cout << "Size of NvbloxNodeParams has changed. Add tests below for any new parameters and "
      "update "
      "kExpectedParamSize to "
              << sizeof(NvbloxNodeParams) << std::endl;
    EXPECT_TRUE(false);
  }

  const char * argv[1] = {"node"};
  rclcpp::init(1, argv);
  auto node = std::make_shared<rclcpp::Node>("node", rclcpp::NodeOptions());

  NvbloxNodeParams params;
  initializeRosParams(node.get(), &params);

  testStringParam(node.get(), params.global_frame);
  testStringParam(node.get(), params.pose_frame);
  testStringParam(node.get(), params.map_clearing_frame_id);
  testStringParam(node.get(), params.slice_visualization_attachment_frame_id);

  testParam<bool>(node.get(), params.print_timings_to_console);
  testParam<bool>(node.get(), params.print_rates_to_console);
  testParam<bool>(node.get(), params.print_delays_to_console);
  testParam<bool>(node.get(), params.enable_mesh_markers);
  testParam<bool>(node.get(), params.use_non_equal_vertical_fov_lidar_params);
  testParam<bool>(node.get(), params.publish_esdf_distance_slice);
  testParam<bool>(node.get(), params.use_color);
  testParam<bool>(node.get(), params.use_depth);
  testParam<bool>(node.get(), params.use_lidar);
  testParam<bool>(node.get(), params.layer_visualization_undo_gamma_correction);

  testParam<int>(node.get(), params.maximum_sensor_message_queue_length);
  testParam<int>(node.get(), params.back_projection_subsampling);
  testParam<int>(node.get(), params.tick_period_ms);
  testParam<int>(node.get(), params.print_statistics_on_console_period_ms);
  testParam<int>(node.get(), params.num_cameras);
  testParam<int>(node.get(), params.lidar_width);
  testParam<int>(node.get(), params.lidar_height);

  testParam<float>(node.get(), params.lidar_vertical_fov_rad);
  testParam<float>(node.get(), params.voxel_size);
  testParam<float>(node.get(), params.min_angle_below_zero_elevation_rad);
  testParam<float>(node.get(), params.max_angle_above_zero_elevation_rad);
  testParam<float>(node.get(), params.slice_visualization_side_length);
  testParam<float>(node.get(), params.layer_visualization_max_tsdf_distance_m);
  testParam<float>(node.get(), params.layer_visualization_min_tsdf_weight);
  testParam<float>(node.get(), params.layer_visualization_exclusion_height_m);
  testParam<float>(node.get(), params.layer_visualization_exclusion_radius_m);
  testParam<float>(node.get(), params.integrate_depth_rate_hz);
  testParam<float>(node.get(), params.integrate_color_rate_hz);
  testParam<float>(node.get(), params.integrate_lidar_rate_hz);
  testParam<float>(node.get(), params.update_mesh_rate_hz);
  testParam<float>(node.get(), params.update_esdf_rate_hz);
  testParam<float>(node.get(), params.publish_layer_rate_hz);
  testParam<float>(node.get(), params.decay_tsdf_rate_hz);
  testParam<float>(node.get(), params.decay_dynamic_occupancy_rate_hz);
  testParam<float>(node.get(), params.clear_map_outside_radius_rate_hz);
  testParam<float>(node.get(), params.esdf_and_gradients_unobserved_value);
  testParam<float>(node.get(), params.map_clearing_radius_m);
  testParam<float>(node.get(), params.max_back_projection_distance);

  testStringParam(node.get(), params.mapping_type_str);

  EXPECT_EQ(params.mapping_type, MappingType::kStaticTsdf);
  EXPECT_EQ(params.esdf_mode, EsdfMode::k2D);
}

}  // namespace nvblox

int main(int argc, char ** argv)
{
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
