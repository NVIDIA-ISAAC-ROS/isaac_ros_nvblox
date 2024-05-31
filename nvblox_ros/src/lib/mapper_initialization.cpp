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

#include "nvblox_ros/mapper_initialization.hpp"

#include <nvblox/integrators/weighting_function.h>

#include <string>

namespace nvblox
{

WeightingFunctionType weighting_function_type_from_string(
  const std::string & weighting_function_str,
  rclcpp::Node * node)
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
  } else if (weighting_function_str == "linear_with_max") {
    return WeightingFunctionType::kLinearWithMax;
  } else {
    RCLCPP_WARN_STREAM(
      node->get_logger(),
      "Requested weighting function: \""
        << weighting_function_str << "\" not recognized. Defaulting to: "
        << kProjectiveIntegratorWeightingModeParamDesc.default_value);
    return kProjectiveIntegratorWeightingModeParamDesc.default_value;
  }
}

MappingType mapping_type_from_string(const std::string & mapping_type_str, rclcpp::Node * node)
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
      node->get_logger(), "Requested mapping type: \""
        << mapping_type_str
        << "\" not recognized. Defaulting to: "
        << toString(MappingType::kStaticTsdf));
    return MappingType::kStaticTsdf;
  }
}

void declareMapperParameters(const std::string & mapper_name, rclcpp::Node * node)
{
  // ======= MAPPER =======
  // depth preprocessing
  declareParameter<bool>(mapper_name, kDoDepthPrepocessingParamDesc, node);
  declareParameter<int>(mapper_name, kDepthPreprocessingNumDilationsParamDesc, node);

  // mesh streaming
  declareParameter<float>(mapper_name, kMeshBandwidthLimitMbpsParamDesc, node);
  // 2D esdf slice
  declareParameter<float>(mapper_name, kEsdfSliceMinHeightParamDesc, node);
  declareParameter<float>(mapper_name, kEsdfSliceMaxHeightParamDesc, node);
  declareParameter<float>(mapper_name, kEsdfSliceHeightParamDesc, node);
  // Decay
  declareParameter<bool>(mapper_name, kExcludeLastViewFromDecayParamDesc, node);

  // ======= PROJECTIVE INTEGRATOR (TSDF/COLOR/OCCUPANCY) =======
  declareParameter<float>(mapper_name, kProjectiveIntegratorMaxIntegrationDistanceMParamDesc, node);
  declareParameter<float>(
    mapper_name, kLidarProjectiveIntegratorMaxIntegrationDistanceMParamDesc,
    node);
  declareParameter<float>(mapper_name, kProjectiveIntegratorTruncationDistanceVoxParamDesc, node);
  declareParameter<WeightingFunctionType, std::string>(
    mapper_name, kProjectiveIntegratorWeightingModeParamDesc, node,
    [](WeightingFunctionType default_value) {return to_string(default_value);});
  declareParameter<float>(mapper_name, kProjectiveIntegratorMaxWeightParamDesc, node);
  // ======= OCCUPANCY INTEGRATOR =======
  declareParameter<float>(mapper_name, kFreeRegionOccupancyProbabilityParamDesc, node);
  declareParameter<float>(mapper_name, kOccupiedRegionOccupancyProbabilityParamDesc, node);
  declareParameter<float>(mapper_name, kUnobservedRegionOccupancyProbabilityParamDesc, node);
  declareParameter<float>(mapper_name, kOccupiedRegionHalfWidthMParamDesc, node);
  // ======= ESDF INTEGRATOR =======
  declareParameter<float>(mapper_name, kEsdfIntegratorMinWeightParamDesc, node);
  declareParameter<float>(mapper_name, kEsdfIntegratorMaxSiteDistanceVoxParamDesc, node);
  declareParameter<float>(mapper_name, kEsdfIntegratorMaxDistanceMParamDesc, node);
  // ======= MESH INTEGRATOR =======
  declareParameter<float>(mapper_name, kMeshIntegratorMinWeightParamDesc, node);
  declareParameter<bool>(mapper_name, kMeshIntegratorWeldVerticesParamDesc, node);
  // ======= TSDF DECAY INTEGRATOR =======
  declareParameter<float>(mapper_name, kTsdfDecayFactorParamDesc, node);
  declareParameter<float>(mapper_name, kTsdfDecayedWeightThresholdDesc, node);
  declareParameter<bool>(mapper_name, kTsdfSetFreeDistanceOnDecayedDesc, node);
  declareParameter<float>(mapper_name, kTsdfDecayedFreeDistanceVoxDesc, node);
  declareParameter<bool, bool>(
    mapper_name, kDecayIntegratorBaseDeallocateDecayedBlocks, node,
    DefaultParamConverter<bool>(), "tsdf_deallocate_on_decayed");

  // ======= OCCUPANCY DECAY INTEGRATOR =======
  declareParameter<float>(mapper_name, kFreeRegionDecayProbabilityParamDesc, node);
  declareParameter<float>(mapper_name, kOccupiedRegionDecayProbabilityParamDesc, node);
  declareParameter<bool, bool>(
    mapper_name, kDecayIntegratorBaseDeallocateDecayedBlocks, node,
    DefaultParamConverter<bool>(), "occupancy_deallocate_on_decayed");

  // ======= FREESPACE INTEGRATOR =======
  declareParameter<float>(mapper_name, kMaxTsdfDistanceForOccupancyMParamDesc, node);
  declareParameter<nvblox::Time, int64_t>(
    mapper_name, kMaxUnobservedToKeepConsecutiveOccupancyMsParamDesc, node);
  declareParameter<nvblox::Time, int64_t>(
    mapper_name,
    kMinDurationSinceOccupiedForFreespaceMsParamDesc, node);
  declareParameter<nvblox::Time, int64_t>(
    mapper_name, kMinConsecutiveOccupancyDurationForResetMsParamDesc, node);
  declareParameter<bool>(mapper_name, kCheckNeighborhoodParamDesc, node);
  // ======= MESH STREAMER =======
  declareParameter<float>(mapper_name, kMeshStreamerExclusionHeightMParamDesc, node);
  declareParameter<float>(mapper_name, kMeshStreamerExclusionRadiusMParamDesc, node);
}

MapperParams getMapperParamsFromROS(const std::string & mapper_name, rclcpp::Node * node)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Initialize Mapper:");

  MapperParams params;

  // ======= MAPPER =======
  // depth preprocessing
  set_mapper_parameter<bool>(
    mapper_name, kDoDepthPrepocessingParamDesc.name,
    [&](auto value) {params.do_depth_preprocessing = value;}, node);
  set_mapper_parameter<int>(
    mapper_name, kDepthPreprocessingNumDilationsParamDesc.name,
    [&](auto value) {params.depth_preprocessing_num_dilations = value;}, node);
  // mesh streaming
  set_mapper_parameter<float>(
    mapper_name, kMeshBandwidthLimitMbpsParamDesc.name,
    [&](auto value) {params.mesh_bandwidth_limit_mbps = value;}, node);
  // 2D esdf slice
  set_mapper_parameter<float>(
    mapper_name, kEsdfSliceMinHeightParamDesc.name,
    [&](auto value) {params.esdf_slice_min_height = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, kEsdfSliceMaxHeightParamDesc.name,
    [&](auto value) {params.esdf_slice_max_height = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, kEsdfSliceHeightParamDesc.name,
    [&](auto value) {params.esdf_slice_height = value;}, node);
  // Decay
  set_mapper_parameter<bool>(
    mapper_name, kExcludeLastViewFromDecayParamDesc.name,
    [&](auto value) {params.exclude_last_view_from_decay = value;}, node);

  // ======= PROJECTIVE INTEGRATOR (TSDF/COLOR/OCCUPANCY) =======
  // max integration distance
  set_mapper_parameter<float>(
    mapper_name, kProjectiveIntegratorMaxIntegrationDistanceMParamDesc.name,
    [&](auto value) {params.projective_integrator_max_integration_distance_m = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, kLidarProjectiveIntegratorMaxIntegrationDistanceMParamDesc.name,
    [&](auto value) {params.lidar_projective_integrator_max_integration_distance_m = value;},
    node);
  // truncation distance
  set_mapper_parameter<float>(
    mapper_name, kProjectiveIntegratorTruncationDistanceVoxParamDesc.name,
    [&](auto value) {params.projective_integrator_truncation_distance_vox = value;}, node);
  // weighting
  set_mapper_parameter<std::string>(
    mapper_name, kProjectiveIntegratorWeightingModeParamDesc.name,
    [&](auto value) {
      const WeightingFunctionType weight_mode = weighting_function_type_from_string(value, node);
      params.projective_integrator_weighting_mode = weight_mode;
    },
    node);
  // max weight
  set_mapper_parameter<float>(
    mapper_name, kProjectiveIntegratorMaxWeightParamDesc.name,
    [&](auto value) {params.projective_integrator_max_weight = value;}, node);

  // ======= OCCUPANCY INTEGRATOR =======
  set_mapper_parameter<float>(
    mapper_name, kFreeRegionOccupancyProbabilityParamDesc.name,
    [&](auto value) {params.free_region_occupancy_probability = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, kOccupiedRegionOccupancyProbabilityParamDesc.name,
    [&](auto value) {params.occupied_region_occupancy_probability = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, kUnobservedRegionOccupancyProbabilityParamDesc.name,
    [&](auto value) {params.unobserved_region_occupancy_probability = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, kOccupiedRegionHalfWidthMParamDesc.name,
    [&](auto value) {params.occupied_region_half_width_m = value;}, node);

  // ======= ESDF INTEGRATOR =======
  set_mapper_parameter<float>(
    mapper_name, kEsdfIntegratorMinWeightParamDesc.name,
    [&](auto value) {params.esdf_integrator_min_weight = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, kEsdfIntegratorMaxSiteDistanceVoxParamDesc.name,
    [&](auto value) {params.esdf_integrator_max_site_distance_vox = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, kEsdfIntegratorMaxDistanceMParamDesc.name,
    [&](auto value) {params.esdf_integrator_max_distance_m = value;}, node);

  // ======= MESH INTEGRATOR =======
  set_mapper_parameter<float>(
    mapper_name, kMeshIntegratorMinWeightParamDesc.name,
    [&](auto value) {params.mesh_integrator_min_weight = value;}, node);
  set_mapper_parameter<bool>(
    mapper_name, kMeshIntegratorWeldVerticesParamDesc.name,
    [&](auto value) {params.mesh_integrator_weld_vertices = value;}, node);

  // ======= TSDF DECAY INTEGRATOR =======
  set_mapper_parameter<float>(
    mapper_name, kTsdfDecayFactorParamDesc.name,
    [&](auto value) {params.tsdf_decay_factor = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, kTsdfDecayedWeightThresholdDesc.name,
    [&](auto value) {params.tsdf_decayed_weight_threshold = value;}, node);
  set_mapper_parameter<bool>(
    mapper_name, kTsdfSetFreeDistanceOnDecayedDesc.name,
    [&](auto value) {params.tsdf_set_free_distance_on_decayed = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, kTsdfDecayedFreeDistanceVoxDesc.name,
    [&](auto value) {params.tsdf_decayed_free_distance_vox = value;}, node);
  set_mapper_parameter<bool>(
    mapper_name, "tsdf_deallocate_on_decayed",
    [&](auto value) {params.tsdf_deallocate_decayed_blocks = value;}, node);

  // ======= OCCUPANCY DECAY INTEGRATOR =======
  set_mapper_parameter<float>(
    mapper_name, kFreeRegionDecayProbabilityParamDesc.name,
    [&](auto value) {params.free_region_decay_probability = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, kOccupiedRegionDecayProbabilityParamDesc.name,
    [&](auto value) {params.occupied_region_decay_probability = value;}, node);
  set_mapper_parameter<bool>(
    mapper_name, "occupancy_deallocate_on_decayed",
    [&](auto value) {params.occupancy_deallocate_decayed_blocks = value;}, node);

  // ======= FREESPACE INTEGRATOR =======
  set_mapper_parameter<float>(
    mapper_name, kMaxTsdfDistanceForOccupancyMParamDesc.name,
    [&](auto value) {params.max_tsdf_distance_for_occupancy_m = value;}, node);
  set_mapper_parameter<int64_t>(
    mapper_name, kMaxUnobservedToKeepConsecutiveOccupancyMsParamDesc.name,
    [&](auto value) {params.max_unobserved_to_keep_consecutive_occupancy_ms = Time(value);},
    node);
  set_mapper_parameter<int64_t>(
    mapper_name, kMinDurationSinceOccupiedForFreespaceMsParamDesc.name,
    [&](auto value) {params.min_duration_since_occupied_for_freespace_ms = Time(value);}, node);
  set_mapper_parameter<int64_t>(
    mapper_name, kMinConsecutiveOccupancyDurationForResetMsParamDesc.name,
    [&](auto value) {params.min_consecutive_occupancy_duration_for_reset_ms = Time(value);},
    node);
  set_mapper_parameter<bool>(
    mapper_name, kCheckNeighborhoodParamDesc.name,
    [&](auto value) {params.check_neighborhood = value;}, node);

  // ======= MESH STREAMER =======
  set_mapper_parameter<float>(
    mapper_name, kMeshStreamerExclusionHeightMParamDesc.name,
    [&](auto value) {params.mesh_streamer_exclusion_height_m = value;}, node);
  set_mapper_parameter<float>(
    mapper_name, kMeshStreamerExclusionRadiusMParamDesc.name,
    [&](auto value) {params.mesh_streamer_exclusion_radius_m = value;}, node);

  return params;
}

}  // namespace nvblox
