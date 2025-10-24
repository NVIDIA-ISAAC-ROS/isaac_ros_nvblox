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

#ifndef NVBLOX_ROS__VISUALIZATION_HPP_
#define NVBLOX_ROS__VISUALIZATION_HPP_

#include <nvblox/nvblox.h>

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace nvblox
{

enum class HeightLimitMarkerType { kTopHeightLimit, kBottomHeightLimit };

/// @brief Generate a marker for visualizing a plane in 3D space.
/// @param T_G_PB Transformation from the plane's center to the global frame.
/// @param plane_G The plane to be visualized in the global frame.
/// @param visualization_side_length The side length of the square plane to visualize.
/// @param timestamp Timestamp for the marker message.
/// @param global_frame_id Frame in which the plane should be visualized.
/// @return A visualization marker representing the plane.
visualization_msgs::msg::Marker planeToMarker(
  const Transform & T_G_PB, const Plane & plane_G,
  float visualization_side_length, const rclcpp::Time & timestamp,
  const std::string & global_frame_id);

/// @brief Generate a marker for visualizing a height limit as a plane.
/// @param T_G_PB Where to center the plane in x/y-direction.
/// @param visualization_side_length Side length of the plane.
/// @param timestamp Timestamp for the marker message.
/// @param global_frame_id Frame the plane should be in.
/// @param height The height limit in meters.
/// @param height_limit_type The limit type for coloring/namespacing.
/// @return A visualization marker representing the height limit as a plane.
visualization_msgs::msg::Marker heightLimitToMarker(
  const Transform & T_G_PB, const float visualization_side_length,
  const rclcpp::Time & timestamp, const std::string & global_frame_id,
  const float height, const HeightLimitMarkerType height_limit_type);

/// @brief Generate a marker for visualizing a bounding box.
/// @param min_corner The minimal corner of the bounding box.
/// @param max_corner The maximal corner of the bounding box.
/// @param timestamp Timestamp for the marker message.
/// @param global_frame_id Frame the bounding box should be in.
/// @return A visualization marker representing a bounding box.
visualization_msgs::msg::Marker boundingBoxToMarker(
  const Vector3f & min_corner,
  const Vector3f & max_corner, const rclcpp::Time & timestamp,
  const std::string & global_frame_id);

/// @brief Generate a marker array from a list of BoundingShapes.
/// @param shapes List of shapes.
/// @param timestamp Timestamp for the markers.
/// @param global_frame_id Frame the shapes are defined in.
/// @param logger ROS logger.
/// @return A marker array representing the shapes.
visualization_msgs::msg::MarkerArray boundingShapesToMarker(
  const std::vector<BoundingShape> & shapes,
  const rclcpp::Time & timestamp,
  const std::string & global_frame_id, rclcpp::Logger logger);

}  // namespace nvblox

#endif  // NVBLOX_ROS__VISUALIZATION_HPP_
