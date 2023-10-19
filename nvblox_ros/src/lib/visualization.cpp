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

#include <nvblox/core/types.h>

#include <string>
#include <vector>

#include "nvblox_ros/visualization.hpp"

namespace nvblox
{

visualization_msgs::msg::Marker sliceLimitsToMarker(
  const Transform & T_G_PB, const float slice_visualization_side_length,
  const rclcpp::Time & timestamp, const std::string & global_frame_id,
  const float min_height, const float max_height)
{
  // Corners of the plane in the plane-body frame.
  // NOTE: We attach the z value later because this is specified in the odom
  // frame.
  const float square_half_side_length_m =
    slice_visualization_side_length / 2.0f;
  Vector3f p0_PB(square_half_side_length_m, square_half_side_length_m, 0.0f);
  Vector3f p1_PB(-square_half_side_length_m, square_half_side_length_m, 0.0f);
  Vector3f p2_PB(square_half_side_length_m, -square_half_side_length_m, 0.0f);
  Vector3f p3_PB(-square_half_side_length_m, -square_half_side_length_m, 0.0f);

  // 6 triangle corners ([0,1,2], [1,2,3])
  std::vector<Vector3f> vertices_PB_vec{p0_PB, p1_PB, p2_PB,
    p1_PB, p2_PB, p3_PB};

  // Vertices in the global coordinate plane
  std::vector<Vector3f> vertices_G_vec;

  // Bottom plane
  for (const Vector3f & vertex_PB : vertices_PB_vec) {
    Vector3f vertex_G = T_G_PB * vertex_PB;
    vertex_G.z() = min_height;
    vertices_G_vec.push_back(vertex_G);
  }
  // Top plane
  for (const Vector3f & vertex_PB : vertices_PB_vec) {
    Vector3f vertex_G = T_G_PB * vertex_PB;
    vertex_G.z() = max_height;
    vertices_G_vec.push_back(vertex_G);
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = global_frame_id;
  marker.header.stamp = timestamp;
  marker.ns = "slice_limits";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 0.25;

  for (const Vector3f p_G : vertices_G_vec) {
    geometry_msgs::msg::Point msg;
    msg.x = p_G.x();
    msg.y = p_G.y();
    msg.z = p_G.z();
    marker.points.push_back(msg);
  }

  // Bottom color
  for (int i = 0; i < 6; i++) {
    std_msgs::msg::ColorRGBA color_msg;
    color_msg.a = 0.8;
    color_msg.g = 1.0;
    marker.colors.push_back(color_msg);
  }

  // Top color
  for (int i = 0; i < 6; i++) {
    std_msgs::msg::ColorRGBA color_msg;
    color_msg.a = 0.8;
    color_msg.r = 1.0;
    marker.colors.push_back(color_msg);
  }

  return marker;
}

}  // namespace nvblox
