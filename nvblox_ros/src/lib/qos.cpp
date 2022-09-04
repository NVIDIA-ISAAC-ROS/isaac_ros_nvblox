/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "nvblox_ros/qos.hpp"

#include <algorithm>
#include <string>

namespace nvblox
{

rmw_qos_profile_t parseQoSString(const std::string & str)
{
  std::string profile = str;
  // Convert to upper case.
  std::transform(profile.begin(), profile.end(), profile.begin(), ::toupper);

  if (profile == "SYSTEM_DEFAULT") {
    return rmw_qos_profile_system_default;
  }
  if (profile == "DEFAULT") {
    return rmw_qos_profile_default;
  }
  if (profile == "PARAMETER_EVENTS") {
    return rmw_qos_profile_parameter_events;
  }
  if (profile == "SERVICES_DEFAULT") {
    return rmw_qos_profile_services_default;
  }
  if (profile == "PARAMETERS") {
    return rmw_qos_profile_parameters;
  }
  if (profile == "SENSOR_DATA") {
    return rmw_qos_profile_sensor_data;
  }
  RCLCPP_WARN_STREAM(
    rclcpp::get_logger("parseQoSString"),
    "Unknown QoS profile: " << profile << ". Returning profile: DEFAULT");
  return rmw_qos_profile_default;
}

}  // namespace nvblox
