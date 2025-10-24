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

#include "nvblox_ros/node_params.hpp"

namespace nvblox
{

template<typename T> void testParam(rclcpp::Node * node, Param<T> & param)
{
  T from_ros{};
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

void testParamSize(const int expected_param_size, const int actual_param_size)
{
  if (actual_param_size != expected_param_size) {
    std::cout << "Size of parameter struct has changed. Add tests for any new parameters and "
      "update expected_param_size to "
              << actual_param_size << std::endl;
    EXPECT_TRUE(false);
  }
}

TEST(BaseNodeParams, initialize) {
  constexpr size_t kExpectedParamSize = 480;
  testParamSize(kExpectedParamSize, sizeof(BaseNodeParams));

  auto node = std::make_shared<rclcpp::Node>("node", rclcpp::NodeOptions());

  BaseNodeParams params;
  initializeBaseNodeParams(node.get(), &params);

  testStringParam(node.get(), params.global_frame);
  testParam<float>(node.get(), params.voxel_size);
  testParam<float>(node.get(), params.max_back_projection_distance);
  testParam<float>(node.get(), params.layer_visualization_min_tsdf_weight);
  testParam<float>(node.get(), params.layer_visualization_exclusion_height_m);
  testParam<float>(node.get(), params.layer_visualization_exclusion_radius_m);
  testParam<float>(node.get(), params.layer_streamer_bandwidth_limit_mbps);

  testStringParam(node.get(), params.esdf_mode_str);
  EXPECT_EQ(params.esdf_mode, EsdfMode::k2D);
  testStringParam(node.get(), params.mapping_type_str);
  EXPECT_EQ(params.mapping_type, MappingType::kStaticTsdf);
}

TEST(NvbloxNodeParams, initialize) {
  constexpr size_t kExpectedParamSize = 2096;
  testParamSize(kExpectedParamSize, sizeof(NvbloxNodeParams));

  auto node = std::make_shared<rclcpp::Node>("node", rclcpp::NodeOptions());

  NvbloxNodeParams params;
  initializeNvbloxNodeParams(node.get(), &params);

  EXPECT_EQ(params.cuda_stream_type, CudaStreamType::kBlocking);

  testStringParam(node.get(), params.pose_frame);
  testStringParam(node.get(), params.map_clearing_frame_id);
  testStringParam(node.get(), params.after_shutdown_map_save_path);
  testStringParam(node.get(), params.esdf_slice_bounds_visualization_attachment_frame_id);
  testStringParam(node.get(), params.workspace_height_bounds_visualization_attachment_frame_id);
  testStringParam(node.get(), params.ground_plane_visualization_attachment_frame_id);
  testParam(node.get(), params.ground_plane_visualization_side_length);
  testParam<bool>(node.get(), params.print_timings_to_console);
  testParam<bool>(node.get(), params.print_rates_to_console);
  testParam<bool>(node.get(), params.print_delays_to_console);
  testParam<bool>(node.get(), params.print_queue_drops_to_console);
  testParam<bool>(node.get(), params.use_non_equal_vertical_fov_lidar_params);
  testParam<bool>(node.get(), params.publish_esdf_distance_slice);
  testParam<bool>(node.get(), params.use_color);
  testParam<bool>(node.get(), params.use_lidar);
  testParam<bool>(node.get(), params.layer_visualization_undo_gamma_correction);
  testParam<bool>(node.get(), params.use_segmentation);

  testParam<int>(node.get(), params.maximum_input_queue_length);
  testParam<int>(node.get(), params.back_projection_subsampling);
  testParam<int>(node.get(), params.tick_period_ms);
  testParam<int>(node.get(), params.print_statistics_on_console_period_ms);
  testParam<int>(node.get(), params.num_cameras);
  testParam<int>(node.get(), params.lidar_width);
  testParam<int>(node.get(), params.lidar_height);

  testParam<float>(node.get(), params.lidar_vertical_fov_rad);
  testParam<float>(node.get(), params.lidar_min_valid_range_m);
  testParam<float>(node.get(), params.lidar_max_valid_range_m);
  testParam<float>(node.get(), params.min_angle_below_zero_elevation_rad);
  testParam<float>(node.get(), params.max_angle_above_zero_elevation_rad);
  testParam<float>(node.get(), params.esdf_slice_bounds_visualization_side_length);
  testParam<float>(node.get(), params.workspace_height_bounds_visualization_side_length);
  testParam<float>(node.get(), params.integrate_depth_rate_hz);
  testParam<float>(node.get(), params.integrate_color_rate_hz);
  testParam<float>(node.get(), params.integrate_lidar_rate_hz);
  testParam<float>(node.get(), params.update_mesh_rate_hz);
  testParam<float>(node.get(), params.update_esdf_rate_hz);
  testParam<float>(node.get(), params.publish_layer_rate_hz);
  testParam<float>(node.get(), params.publish_debug_vis_rate_hz);
  testParam<float>(node.get(), params.decay_tsdf_rate_hz);
  testParam<float>(node.get(), params.decay_dynamic_occupancy_rate_hz);
  testParam<float>(node.get(), params.clear_map_outside_radius_rate_hz);
  testParam<float>(node.get(), params.esdf_and_gradients_unobserved_value);
  testParam<float>(node.get(), params.map_clearing_radius_m);
}

TEST(FuserNodeParams, initialize) {
  constexpr size_t kExpectedParamSize = 920;
  testParamSize(kExpectedParamSize, sizeof(FuserNodeParams));

  auto node = std::make_shared<rclcpp::Node>("node", rclcpp::NodeOptions());

  FuserNodeParams params;
  initializeFuserNodeParams(node.get(), &params);

  testStringParam(node.get(), params.dataset_path);
  testParam<int>(node.get(), params.number_of_frames_to_integrate);
  testParam<bool>(node.get(), params.update_on_key);

  testStringParam(node.get(), params.dataset_type_str);
  EXPECT_EQ(params.dataset_type, RosDatasetType::kThreedMatch);

  testStringParam(node.get(), params.depth_topic);
  testStringParam(node.get(), params.depth_camera_info_topic);
  testStringParam(node.get(), params.color_topic);
  testStringParam(node.get(), params.color_camera_info_topic);
  testParam<float>(node.get(), params.tf_lead_time_s);
}

}  // namespace nvblox

int main(int argc, char ** argv)
{
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  testing::InitGoogleTest(&argc, argv);

  const char * rclcpp_argv[1] = {"node"};
  rclcpp::init(1, rclcpp_argv);

  return RUN_ALL_TESTS();
}
