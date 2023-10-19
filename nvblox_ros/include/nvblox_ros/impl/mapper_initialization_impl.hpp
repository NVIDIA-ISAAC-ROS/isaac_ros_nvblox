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

#ifndef NVBLOX_ROS__IMPL__MAPPER_INITIALIZATION_IMPL_HPP_
#define NVBLOX_ROS__IMPL__MAPPER_INITIALIZATION_IMPL_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace nvblox
{

template<typename T>
void set_mapper_parameter(
  const std::string & mapper_name,
  const std::string & parameter_name,
  std::function<void(T)> parameter_setter,
  rclcpp::Node * node)
{
  T parameter_value;
  const std::string full_name = mapper_name + "." + parameter_name;
  if (node->get_parameter<T>(full_name, parameter_value)) {
    // Print non default values
    RCLCPP_INFO_STREAM(
      node->get_logger(),
      full_name << ": " << parameter_value);
    // Set the mapper parameter
    parameter_setter(parameter_value);
  }
}

}  // namespace nvblox

#endif  // NVBLOX_ROS__IMPL__MAPPER_INITIALIZATION_IMPL_HPP_
