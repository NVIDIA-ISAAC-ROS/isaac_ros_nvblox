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

#include <nvblox/core/types.h>

#include <string>
#include <vector>

#include "nvblox_ros/visualization.hpp"

namespace nvblox
{

inline std::string toString(HeightLimitMarkerType marker_type)
{
  switch (marker_type) {
    case HeightLimitMarkerType::kTopHeightLimit:
      return "top_height_limit";
      break;
    case HeightLimitMarkerType::kBottomHeightLimit:
      return "bottom_height_limit";
      break;
    default:
      return "unknown_type";
  }
}

visualization_msgs::msg::Marker planeToMarker(
  const Transform & T_G_PB, const Plane & plane_G,
  float visualization_side_length, const rclcpp::Time & timestamp,
  const std::string & global_frame_id)
{
  // Get four points in the respective body frame (B). The Body to Global transformation
  // (T_G_PB) is expected to rotate around z (yaw) only.
  // Then transform the body point into the global frame (G) in which we extract the z height
  // from the plane.
  const float square_half_side_length_m = visualization_side_length / 2.0f;

  const Vector3f p0_B{square_half_side_length_m, square_half_side_length_m, 0.0};
  Vector3f p0_G = T_G_PB * p0_B;
  p0_G.z() = plane_G.getHeightAtXY({p0_G.x(), p0_G.y()});

  const Vector3f p1_B{-square_half_side_length_m, square_half_side_length_m, 0.0};
  Vector3f p1_G = T_G_PB * p1_B;
  p1_G.z() = plane_G.getHeightAtXY({p1_G.x(), p1_G.y()});

  const Vector3f p2_B{square_half_side_length_m, -square_half_side_length_m, 0.0};
  Vector3f p2_G = T_G_PB * p2_B;
  p2_G.z() = plane_G.getHeightAtXY({p2_G.x(), p2_G.y()});

  const Vector3f p3_B{-square_half_side_length_m, -square_half_side_length_m, 0.0};
  Vector3f p3_G = T_G_PB * p3_B;
  p3_G.z() = plane_G.getHeightAtXY({p3_G.x(), p3_G.y()});

  // Create marker message
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = global_frame_id;
  marker.header.stamp = timestamp;
  marker.ns = "ground_plane";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // 6 triangle corners ([0,1,2], [1,2,3])
  const std::array<Vector3f, 6> vertices_G_vec{p0_G, p1_G, p2_G, p1_G, p2_G, p3_G};
  for (const Vector3f & vertex_G : vertices_G_vec) {
    geometry_msgs::msg::Point msg;
    msg.x = vertex_G.x();
    msg.y = vertex_G.y();
    msg.z = vertex_G.z();
    marker.points.push_back(msg);

    // Add color to point
    std_msgs::msg::ColorRGBA color_msg;
    color_msg.g = 1.0;
    color_msg.a = 1.0;
    marker.colors.push_back(color_msg);
  }
  return marker;
}


visualization_msgs::msg::Marker
heightLimitToMarker(
  const Transform & T_G_PB, const float visualization_side_length,
  const rclcpp::Time & timestamp, const std::string & global_frame_id,
  const float height, const HeightLimitMarkerType height_limit_type)
{
  // Corners of the plane in the plane-body frame.
  // NOTE: We attach the z value later because this is specified in the odom
  // frame.
  const float square_half_side_length_m = visualization_side_length / 2.0f;
  Vector3f p0_PB(square_half_side_length_m, square_half_side_length_m, 0.0f);
  Vector3f p1_PB(-square_half_side_length_m, square_half_side_length_m, 0.0f);
  Vector3f p2_PB(square_half_side_length_m, -square_half_side_length_m, 0.0f);
  Vector3f p3_PB(-square_half_side_length_m, -square_half_side_length_m, 0.0f);

  // 6 triangle corners ([0,1,2], [1,2,3])
  std::vector<Vector3f> vertices_PB_vec{p0_PB, p1_PB, p2_PB, p1_PB, p2_PB, p3_PB};

  // Create marker message
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = global_frame_id;
  marker.header.stamp = timestamp;
  marker.ns = toString(height_limit_type);
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 0.25;

  for (const Vector3f & vertex_PB : vertices_PB_vec) {
    // Transform to global frame
    Vector3f vertex_G = T_G_PB * vertex_PB;

    // Add point with z-height
    geometry_msgs::msg::Point msg;
    msg.x = vertex_G.x();
    msg.y = vertex_G.y();
    msg.z = height;
    marker.points.push_back(msg);

    // Add color to point
    std_msgs::msg::ColorRGBA color_msg;
    if (height_limit_type == HeightLimitMarkerType::kTopHeightLimit) {
      color_msg.r = 1.0;
    } else {
      color_msg.g = 1.0;
    }
    color_msg.a = 0.8;
    marker.colors.push_back(color_msg);
  }

  return marker;
}

visualization_msgs::msg::Marker boundingBoxToMarker(
  const Vector3f & min_corner,
  const Vector3f & max_corner,
  const rclcpp::Time & timestamp,
  const std::string & global_frame_id)
{
  // Putting all vertices into a vector for iterating over the vertices.
  std::vector<Vector3f> vertices = {
    {min_corner},                                        // p_000 -> 0
    {min_corner.x(), min_corner.y(), max_corner.z()},    // p_001 -> 1
    {min_corner.x(), max_corner.y(), min_corner.z()},    // p_010 -> 2
    {min_corner.x(), max_corner.y(), max_corner.z()},    // p_011 -> 3
    {max_corner.x(), min_corner.y(), min_corner.z()},    // p_100 -> 4
    {max_corner.x(), min_corner.y(), max_corner.z()},    // p_101 -> 5
    {max_corner.x(), max_corner.y(), min_corner.z()},    // p_110 -> 6
    {max_corner}                                         // p_111 -> 7
  };

  // Define the 12 edges by connecting the vertices
  std::vector<std::array<int, 2>> edges = {//
                                           // Bottom face edges
    {0, 2},
    {2, 6},
    {6, 4},
    {4, 0},
    // Top face edges
    {1, 3},
    {3, 7},
    {7, 5},
    {5, 1},
    // Side edges connecting top/bottom face
    {0, 1},
    {2, 3},
    {6, 7},
    {4, 5}};

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = global_frame_id;
  marker.header.stamp = timestamp;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.05;
  marker.color.r = 1.0;
  marker.color.a = 0.5;

  geometry_msgs::msg::Point vertex_point_1, vertex_point_2;
  for (const auto & edge : edges) {
    vertex_point_1.x = vertices[edge[0]].x();
    vertex_point_1.y = vertices[edge[0]].y();
    vertex_point_1.z = vertices[edge[0]].z();
    marker.points.push_back(vertex_point_1);

    vertex_point_2.x = vertices[edge[1]].x();
    vertex_point_2.y = vertices[edge[1]].y();
    vertex_point_2.z = vertices[edge[1]].z();
    marker.points.push_back(vertex_point_2);
  }
  return marker;
}

visualization_msgs::msg::MarkerArray boundingShapesToMarker(
  const std::vector<BoundingShape> & shapes, const rclcpp::Time & timestamp,
  const std::string & global_frame_id, rclcpp::Logger logger)
{
  // Delete all previous markers.
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.resize(1);
  marker_array.markers[0].id = 0;
  marker_array.markers[0].action = visualization_msgs::msg::Marker::DELETEALL;

  // Return if there are no shapes to visualize.
  const size_t shape_num = shapes.size();
  if (shape_num == 0) {
    return marker_array;
  }

  // Prepare marker with fields that are equal for all shapes.
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = global_frame_id;
  marker.header.stamp = timestamp;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color.b = 1.0;
  marker.color.a = 0.4;

  // Iterate over shapes.
  marker_array.markers.reserve(shape_num + 1);
  for (size_t i = 0; i < shape_num; i++) {
    const BoundingShape shape = shapes[i];
    switch (shape.type()) {
      case ShapeType::kSphere: {
          marker.type = visualization_msgs::msg::Marker::SPHERE;
          const float diameter = 2.f * shape.sphere().radius();
          const Vector3f center = shape.sphere().center();
          marker.id = i + 1;
          marker.scale.x = diameter;
          marker.scale.y = diameter;
          marker.scale.z = diameter;
          marker.pose.position.x = center.x();
          marker.pose.position.y = center.y();
          marker.pose.position.z = center.z();
          marker_array.markers.push_back(marker);
          break;
        }
      case ShapeType::kAABB: {
          marker.type = visualization_msgs::msg::Marker::CUBE;
          const Vector3f size = shape.aabb().max() - shape.aabb().min();
          const Vector3f center = shape.aabb().min() + 0.5f * size;
          marker.id = i + 1;
          marker.scale.x = size.x();
          marker.scale.y = size.y();
          marker.scale.z = size.z();
          marker.pose.position.x = center.x();
          marker.pose.position.y = center.y();
          marker.pose.position.z = center.z();
          marker_array.markers.push_back(marker);
          break;
        }
      default: {
          RCLCPP_ERROR_STREAM(logger, "ShapeType not implemented: " << shape.type());
          break;
        }
    }
  }
  return marker_array;
}

}  // namespace nvblox
