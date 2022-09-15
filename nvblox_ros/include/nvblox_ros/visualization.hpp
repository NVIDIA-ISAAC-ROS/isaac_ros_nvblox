/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef NVBLOX_ROS__VISUALIZATION_HPP_
#define NVBLOX_ROS__VISUALIZATION_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace nvblox
{

visualization_msgs::msg::Marker sliceLimitsToMarker(
  const Transform & T_G_PB, const float slice_visualization_side_length,
  const rclcpp::Time & timestamp, const std::string & global_frame_id,
  const float min_height, const float max_height);

}  // namespace nvblox

#endif  // NVBLOX_ROS__VISUALIZATION_HPP_
