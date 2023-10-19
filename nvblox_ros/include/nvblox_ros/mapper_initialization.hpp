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

#ifndef NVBLOX_ROS__MAPPER_INITIALIZATION_HPP_
#define NVBLOX_ROS__MAPPER_INITIALIZATION_HPP_

#include <nvblox/mapper/mapper.h>
#include <nvblox/mapper/multi_mapper.h>

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "nvblox_ros/utils.hpp"

namespace nvblox
{

MappingType mapping_type_from_string(
  const std::string & mapping_type_str, rclcpp::Node * node);

void declareMapperParameters(const std::string & mapper_name, rclcpp::Node * node);

template<typename T>
void set_mapper_parameter(
  const std::string & mapper_name,
  const std::string & parameter_name,
  std::function<void(T)> parameter_setter,
  rclcpp::Node * node);

MapperParams getMapperParamsFromROS(const std::string & mapper_name, rclcpp::Node * node);

}  // namespace nvblox

#include "nvblox_ros/impl/mapper_initialization_impl.hpp"

#endif  // NVBLOX_ROS__MAPPER_INITIALIZATION_HPP_
