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

#include "nvblox_ros/mapper_initialization.hpp"

#include <nvblox/integrators/weighting_function.h>

#include <string>

namespace nvblox
{

WeightingFunctionType weighting_function_type_from_string(
  const std::string & weighting_function_str, rclcpp::Node * node)
{
  if (weighting_function_str == "constant") {
    return WeightingFunctionType::kConstantWeight;
  } else if (weighting_function_str == "constant_dropoff") {
    return WeightingFunctionType::kConstantDropoffWeight;
  } else if (weighting_function_str == "inverse_square") {
    return WeightingFunctionType::kInverseSquareWeight;
  } else if (weighting_function_str == "inverse_square_dropoff") {
    return WeightingFunctionType::kInverseSquareDropoffWeight;
  } else if (weighting_function_str == "inverse_square_tsdf_distance_penalty") {
    return WeightingFunctionType::kInverseSquareTsdfDistancePenalty;
  } else {
    RCLCPP_WARN_STREAM(
      node->get_logger(),
      "Requested weighting function: \""
        << weighting_function_str
        << "\" not recognized. Defaulting to: "
        << ProjectiveIntegrator<void>::kDefaultWeightingFunctionType);
    return ProjectiveIntegrator<void>::kDefaultWeightingFunctionType;
  }
}

MappingType mapping_type_from_string(
  const std::string & mapping_type_str, rclcpp::Node * node)
{
  if (mapping_type_str == "static_tsdf") {
    return MappingType::kStaticTsdf;
  } else if (mapping_type_str == "static_occupancy") {
    return MappingType::kStaticOccupancy;
  } else if (mapping_type_str == "dynamic") {
    return MappingType::kDynamic;
  } else if (mapping_type_str == "human_with_static_tsdf") {
    return MappingType::kHumanWithStaticTsdf;
  } else if (mapping_type_str == "human_with_static_occupancy") {
    return MappingType::kHumanWithStaticOccupancy;
  } else {
    RCLCPP_WARN_STREAM(
      node->get_logger(),
      "Requested mapping type: \""
        << mapping_type_str
        << "\" not recognized. Defaulting to: "
        << toString(MappingType::kStaticTsdf));
    return MappingType::kStaticTsdf;
  }
}


void declareMapperParameters(
  const std::string & mapper_name,
  rclcpp::Node * node)
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

  // ======= MAPPER =======
  declareParameterWithoutDefault<bool>(
    mapper_name + ".do_depth_preprocessing", node);
  declareParameterWithoutDefault<int64_t>(
    mapper_name + ".depth_preprocessing_num_dilations", node);
  // ======= PROJECTIVE INTEGRATOR (TSDF/COLOR/OCCUPANCY) =======
  declareParameterWithoutDefault<float>(
    mapper_name + ".projective_integrator_max_integration_distance_m", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".lidar_projective_integrator_max_integration_distance_m", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".projective_integrator_truncation_distance_vox", node);
  declareParameterWithoutDefault<std::string>(
    mapper_name + ".weighting_mode", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".projective_integrator_max_weight", node);
  // ======= OCCUPANCY INTEGRATOR =======
  declareParameterWithoutDefault<float>(
    mapper_name + ".free_region_occupancy_probability", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".occupied_region_occupancy_probability", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".unobserved_region_occupancy_probability", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".occupied_region_half_width_m", node);
  // ======= ESDF INTEGRATOR =======
  declareParameterWithoutDefault<float>(
    mapper_name + ".esdf_integrator_min_weight", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".esdf_integrator_max_site_distance_vox", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".esdf_integrator_max_distance_m", node);
  // ======= MESH INTEGRATOR =======
  declareParameterWithoutDefault<float>(
    mapper_name + ".mesh_integrator_min_weight", node);
  declareParameterWithoutDefault<bool>(
    mapper_name + ".mesh_integrator_weld_vertices", node);
  // ======= TSDF DECAY INTEGRATOR =======
  declareParameterWithoutDefault<float>(
    mapper_name + ".tsdf_decay_factor", node);
  // ======= OCCUPANCY DECAY INTEGRATOR =======
  declareParameterWithoutDefault<float>(
    mapper_name + ".free_region_decay_probability", node);
  declareParameterWithoutDefault<float>(
    mapper_name + ".occupied_region_decay_probability", node);
  // ======= FREESPACE INTEGRATOR =======
  declareParameterWithoutDefault<float>(
    mapper_name + ".max_tsdf_distance_for_occupancy_m", node);
  declareParameterWithoutDefault<int>(
    mapper_name + ".max_unobserved_to_keep_consecutive_occupancy_ms", node);
  declareParameterWithoutDefault<int>(
    mapper_name + ".min_duration_since_occupied_for_freespace_ms", node);
  declareParameterWithoutDefault<int>(
    mapper_name + ".min_consecutive_occupancy_duration_for_reset_ms", node);
  declareParameterWithoutDefault<bool>(
    mapper_name + ".check_neighborhood", node);
}

MapperParams getMapperParamsFromROS(const std::string & mapper_name, rclcpp::Node * node)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Initialize Mapper:");

  MapperParams params;

  // ======= MAPPER =======
  // depth preprocessing
  set_mapper_parameter<bool>(
    mapper_name, "do_depth_preprocessing",
    [&](auto value) {params.do_depth_preprocessing = value;}, node);
  set_mapper_parameter<int64_t>(
    mapper_name, "depth_preprocessing_num_dilations",
    [&](auto value) {params.depth_preprocessing_num_dilations = value;}, node);

  // ======= PROJECTIVE INTEGRATOR (TSDF/COLOR/OCCUPANCY) =======
  // max integration distance
  set_mapper_parameter<float>(
    mapper_name, "projective_integrator_max_integration_distance_m",
    [&](auto value) {
      params.projective_integrator_max_integration_distance_m = value;
    }, node);
  set_mapper_parameter<float>(
    mapper_name, "lidar_projective_integrator_max_integration_distance_m",
    [&](auto value) {
      params.lidar_projective_integrator_max_integration_distance_m = value;
    }, node);
  // truncation distance
  set_mapper_parameter<float>(
    mapper_name, "projective_integrator_truncation_distance_vox",
    [&](auto value) {
      params.projective_integrator_truncation_distance_vox = value;
    }, node);
  // weighting
  set_mapper_parameter<std::string>(
    mapper_name, "weighting_mode",
    [&](auto value) {
      const WeightingFunctionType weight_mode =
      weighting_function_type_from_string(value, node);
      params.projective_integrator_weighting_mode = weight_mode;
    }, node);
  // max weight
  set_mapper_parameter<float>(
    mapper_name, "projective_integrator_max_weight",
    [&](auto value) {params.projective_integrator_max_weight = value;}, node);

  // ======= OCCUPANCY INTEGRATOR =======
  set_mapper_parameter<float>(
    mapper_name, "free_region_occupancy_probability",
    [&](auto value) {params.free_region_occupancy_probability = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, "occupied_region_occupancy_probability",
    [&](auto value) {params.occupied_region_occupancy_probability = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, "unobserved_region_occupancy_probability",
    [&](auto value) {
      params.unobserved_region_occupancy_probability = value;
    }, node);
  set_mapper_parameter<float>(
    mapper_name, "occupied_region_half_width_m",
    [&](auto value) {params.occupied_region_half_width_m = value;}, node);

  // ======= ESDF INTEGRATOR =======
  set_mapper_parameter<float>(
    mapper_name, "esdf_integrator_min_weight",
    [&](auto value) {params.esdf_integrator_min_weight = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, "esdf_integrator_max_site_distance_vox",
    [&](auto value) {params.esdf_integrator_max_site_distance_vox = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, "esdf_integrator_max_distance_m",
    [&](auto value) {params.esdf_integrator_max_distance_m = value;}, node);

  // ======= MESH INTEGRATOR =======
  set_mapper_parameter<float>(
    mapper_name, "mesh_integrator_min_weight",
    [&](auto value) {params.mesh_integrator_min_weight = value;}, node);
  set_mapper_parameter<bool>(
    mapper_name, "mesh_integrator_weld_vertices",
    [&](auto value) {params.mesh_integrator_weld_vertices = value;}, node);

  // ======= TSDF DECAY INTEGRATOR =======
  set_mapper_parameter<float>(
    mapper_name, "tsdf_decay_factor",
    [&](auto value) {params.tsdf_decay_factor = value;}, node);

  // ======= OCCUPANCY DECAY INTEGRATOR =======
  set_mapper_parameter<float>(
    mapper_name, "free_region_decay_probability",
    [&](auto value) {params.free_region_decay_probability = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, "occupied_region_decay_probability",
    [&](auto value) {params.occupied_region_decay_probability = value;}, node);

  // ======= FREESPACE INTEGRATOR =======
  set_mapper_parameter<float>(
    mapper_name, "max_tsdf_distance_for_occupancy_m",
    [&](auto value) {params.max_tsdf_distance_for_occupancy_m = value;}, node);
  set_mapper_parameter<int64_t>(
    mapper_name, "max_unobserved_to_keep_consecutive_occupancy_ms",
    [&](auto value) {
      params.max_unobserved_to_keep_consecutive_occupancy_ms = Time(value);
    }, node);
  set_mapper_parameter<int64_t>(
    mapper_name, "min_duration_since_occupied_for_freespace_ms",
    [&](auto value) {
      params.min_duration_since_occupied_for_freespace_ms = Time(value);
    }, node);
  set_mapper_parameter<int64_t>(
    mapper_name, "min_consecutive_occupancy_duration_for_reset_ms",
    [&](auto value) {
      params.min_consecutive_occupancy_duration_for_reset_ms = Time(value);
    }, node);
  set_mapper_parameter<bool>(
    mapper_name, "check_neighborhood",
    [&](auto value) {params.check_neighborhood = value;}, node);

  return params;
}

}  // namespace nvblox
