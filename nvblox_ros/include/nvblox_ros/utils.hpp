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

#ifndef NVBLOX_ROS__UTILS_HPP_
#define NVBLOX_ROS__UTILS_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "nvblox/mapper/mapper_params.h"

namespace nvblox
{

template<typename ParameterType>
void inline declareParameterWithoutDefault(
  const std::string & name,
  rclcpp::Node * node_ptr)
{
  try {
    node_ptr->declare_parameter<ParameterType>(name);
  } catch (
    const rclcpp::exceptions::UninitializedStaticallyTypedParameterException & ex)
  {
  }
}

// Default conversion from an nvblox parameter to a ROS parameter
template<typename NvbloxParamType, typename RosParamType = NvbloxParamType>
struct DefaultParamConverter
{
  RosParamType operator()(NvbloxParamType param)
  {
    return static_cast<RosParamType>(param);
  }
};

/// Declare a ROS parameter.
///
/// @param name_prefix A prefix added to the param name. Empty string means no prefix.
/// @param desc Description of parameter that defines name, description and default value.
/// @param node Node that the parameter will be added to.
/// @param cast_default_value_to_ros_type Optional functor to cast type to ROS type.
/// @param name_override Optional name override.
template<typename NvbloxParamType, typename RosParamType = NvbloxParamType>
void declareParameter(
  const std::string & name_prefix,
  const typename nvblox::Param<NvbloxParamType>::Description & desc,
  rclcpp::Node * node,
  std::function<RosParamType(NvbloxParamType)> cast_default_value_to_ros_type =
  DefaultParamConverter<NvbloxParamType, RosParamType>(),
  const std::string & name_override = ""
)
{
  const std::string & ros_parameter_name = name_override.empty() ? desc.name : name_override;
  rcl_interfaces::msg::ParameterDescriptor ros_desc;
  ros_desc.description = desc.help_string;
  if (!name_prefix.empty()) {
    node->declare_parameter<RosParamType>(
      name_prefix + "." + ros_parameter_name, cast_default_value_to_ros_type(desc.default_value),
      ros_desc);
  } else {
    node->declare_parameter<RosParamType>(
      ros_parameter_name, cast_default_value_to_ros_type(desc.default_value),
      ros_desc);
  }
}

/// Declare a ROS parameter without prefix.
template<typename NvbloxParamType, typename RosParamType = NvbloxParamType>
void declareParameter(
  const typename nvblox::Param<NvbloxParamType>::Description & desc,
  rclcpp::Node * node,
  std::function<RosParamType(NvbloxParamType)> cast_default_value_to_ros_type =
  DefaultParamConverter<NvbloxParamType, RosParamType>(),
  const std::string & name_override = ""
)
{
  declareParameter("", desc, node, cast_default_value_to_ros_type, name_override);
}

// Transforms a topic name into a safe one (only use alphanumerics, '/', _', '~', '{', or '}'.
std::string makeSafeTopicName(const std::string & topic_name);

}  // namespace nvblox

#endif  // NVBLOX_ROS__UTILS_HPP_
