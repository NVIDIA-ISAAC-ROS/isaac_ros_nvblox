// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "nvblox_ros/mapper_initialization.hpp"

#include <nvblox/integrators/weighting_function.h>

#include <string>

namespace nvblox
{

WeightingFunctionType weighting_function_type_from_string(
  const std::string & weighting_function_str, ros::Node * node)
{
  if (weighting_function_str == "constant") {
    return WeightingFunctionType::kConstantWeight;
  } else if (weighting_function_str == "constant_dropoff") {
    return WeightingFunctionType::kConstantDropoffWeight;
  } else if (weighting_function_str == "inverse_square") {
    return WeightingFunctionType::kInverseSquareWeight;
  } else if (weighting_function_str == "inverse_square_dropoff") {
    return WeightingFunctionType::kInverseSquareDropoffWeight;
  } else {
    ROS_WARN_STREAM("Requested weighting function: \"" <<
        weighting_function_str <<
        "\" not recognized. Defaulting to: " <<
        kDefaultWeightingFunctionType);
    return kDefaultWeightingFunctionType;
  }
}

void declareMapperParameters(
  const std::string & mapper_name,
  ros::Node * node)
{
  // Declare parameters
  // NOTE(alexmillane): We have to use the insane syntax in
  // declareParameterWithoutDefault() order to avoid passing a default value to
  // declare_parameter(). The problem with using a default value is that when we
  // later call node->get_parameter() in initialize_mapper(), it will always
  // return true, even if the user has not set the parameter. To me this appears
  // like a bug in ROS 2. After hours of trying, the insane combination of
  // syntaxes in this function and initialize_mapper() was the only this I found
  // that worked. UPDATE(helzor): Apparently this issue is fixed in later ROS 2
  // versions.
  declareParameterWithoutDefault<float>(
    mapper_name + ".projective_integrator_max_integration_distance_m", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".lidar_projective_integrator_max_integration_distance_m",
    node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".projective_integrator_truncation_distance_vox", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".tsdf_integrator_max_weight", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".free_region_occupancy_probability", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".occupied_region_occupancy_probability", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".unobserved_region_occupancy_probability", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".occupied_region_half_width_m", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".free_region_decay_probability", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".occupied_region_decay_probability", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".mesh_integrator_min_weight", node);
  declareParameterWithoutDefault<bool>(
    mapper_name + ".mesh_integrator_weld_vertices", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".color_integrator_max_integration_distance_m", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".esdf_integrator_min_weight", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".esdf_integrator_max_site_distance_vox", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".esdf_integrator_max_distance_m", node);
  declareParameterWithoutDefault<std::string>(
    mapper_name + ".weighting_mode",
    node);
}

void initializeMapper(
  const std::string & mapper_name, Mapper * mapper_ptr,
  ros::Node * node)
{
  ROS_INFO_STREAM(node-> , "Initialize Mapper:");

  // tsdf or occupancy integrator
  set_mapper_parameter<float>(
    mapper_name, "projective_integrator_max_integration_distance_m",
    [&](auto value) {
      mapper_ptr->tsdf_integrator().max_integration_distance_m(value);
      mapper_ptr->occupancy_integrator().max_integration_distance_m(value);
    },
    node);
  set_mapper_parameter<float>(
    mapper_name, "lidar_projective_integrator_max_integration_distance_m",
    [&](auto value) {
      mapper_ptr->lidar_tsdf_integrator().max_integration_distance_m(value);
      mapper_ptr->lidar_occupancy_integrator().max_integration_distance_m(
        value);
    },
    node);
  set_mapper_parameter<float>(
    mapper_name, "projective_integrator_truncation_distance_vox",
    [&](auto value) {
      mapper_ptr->tsdf_integrator().truncation_distance_vox(value);
      mapper_ptr->occupancy_integrator().truncation_distance_vox(value);
      mapper_ptr->lidar_tsdf_integrator().truncation_distance_vox(value);
      mapper_ptr->lidar_occupancy_integrator().truncation_distance_vox(value);
    },
    node);

  // tsdf and color integrator
  // NOTE(alexmillane): Currently weighting mode does not affect the occupancy
  // integrator.
  set_mapper_parameter<std::string>(
    mapper_name, "weighting_mode",
    [&](auto value) {
      const WeightingFunctionType weight_mode =
      weighting_function_type_from_string(value, node);
      mapper_ptr->tsdf_integrator().weighting_function_type(weight_mode);
      mapper_ptr->color_integrator().weighting_function_type(weight_mode);
    },
    node);

  // tsdf integrator
  set_mapper_parameter<float>(
    mapper_name, "tsdf_integrator_max_weight",
    [&](auto value) {
      mapper_ptr->tsdf_integrator().max_weight(value);
      mapper_ptr->lidar_tsdf_integrator().max_weight(value);
    },
    node);

  // occupancy integrator
  set_mapper_parameter<float>(
    mapper_name, "free_region_occupancy_probability",
    [&](auto value) {
      mapper_ptr->occupancy_integrator().free_region_occupancy_probability(
        value);
      mapper_ptr->lidar_occupancy_integrator()
      .free_region_occupancy_probability(value);
    },
    node);
  set_mapper_parameter<float>(
    mapper_name, "occupied_region_occupancy_probability",
    [&](auto value) {
      mapper_ptr->occupancy_integrator()
      .occupied_region_occupancy_probability(value);
      mapper_ptr->lidar_occupancy_integrator()
      .occupied_region_occupancy_probability(value);
    },
    node);
  set_mapper_parameter<float>(
    mapper_name, "unobserved_region_occupancy_probability",
    [&](auto value) {
      mapper_ptr->occupancy_integrator()
      .unobserved_region_occupancy_probability(value);
      mapper_ptr->lidar_occupancy_integrator()
      .unobserved_region_occupancy_probability(value);
    },
    node);
  set_mapper_parameter<float>(
    mapper_name, "occupied_region_half_width_m",
    [&](auto value) {
      mapper_ptr->occupancy_integrator().occupied_region_half_width_m(value);
      mapper_ptr->lidar_occupancy_integrator().occupied_region_half_width_m(
        value);
    },
    node);

  // occupancy decay
  set_mapper_parameter<float>(
    mapper_name, "free_region_decay_probability",
    [&](auto value) {
      mapper_ptr->occupancy_decay_integrator().free_region_decay_probability(
        value);
    },
    node);
  set_mapper_parameter<float>(
    mapper_name, "occupied_region_decay_probability",
    [&](auto value) {
      mapper_ptr->occupancy_decay_integrator()
      .occupied_region_decay_probability(value);
    },
    node);

  // mesh integrator
  set_mapper_parameter<float>(
    mapper_name, "mesh_integrator_min_weight",
    [&](auto value) {mapper_ptr->mesh_integrator().min_weight(value);},
    node);
  set_mapper_parameter<bool>(
    mapper_name, "mesh_integrator_weld_vertices",
    [&](auto value) {mapper_ptr->mesh_integrator().weld_vertices(value);},
    node);

  // color integrator
  set_mapper_parameter<float>(
    mapper_name, "color_integrator_max_integration_distance_m",
    [&](auto value) {
      mapper_ptr->color_integrator().max_integration_distance_m(value);
    },
    node);

  // esdf integrator
  set_mapper_parameter<float>(
    mapper_name, "esdf_integrator_min_weight",
    [&](auto value) {mapper_ptr->esdf_integrator().min_weight(value);},
    node);
  set_mapper_parameter<float>(
    mapper_name, "esdf_integrator_max_site_distance_vox",
    [&](auto value) {
      mapper_ptr->esdf_integrator().max_site_distance_vox(value);
    },
    node);
  set_mapper_parameter<float>(
    mapper_name, "esdf_integrator_max_distance_m",
    [&](auto value) {mapper_ptr->esdf_integrator().max_distance_m(value);},
    node);
}

}  // namespace nvblox
