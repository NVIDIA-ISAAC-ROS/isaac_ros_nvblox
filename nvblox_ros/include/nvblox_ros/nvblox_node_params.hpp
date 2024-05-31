// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVBLOX_ROS__NVBLOX_NODE_PARAMS_HPP_
#define NVBLOX_ROS__NVBLOX_NODE_PARAMS_HPP_

#include "nvblox/utils/params.h"
#include "nvblox/mapper/multi_mapper.h"

#include <rclcpp/rclcpp.hpp>

namespace nvblox
{

/// Convert degrees to radians
constexpr float degreeToRadians(float degrees)
{
  return degrees * M_PI / 180.F;
}

// ======= ROBOT FRAME PARAMS =======
constexpr StringParam::Description kGlobalFrameParamDesc{
  "global_frame", "odom",
  "The name of the TF frame in which the map is built. "
  "For the RealSense examples, this parameter is exposed as a launch argument."};

constexpr StringParam::Description kPoseFrameParamDesc{
  "pose_frame", "base_link",
  "Only used if use_topic_transforms is set to true. Pose and transform messages will be "
  "interpreted as being in this pose frame, and the remaining transform to the sensor "
  "frame will be looked up on the TF tree."};

// ======= DATA PROCESSING TOGGLES =======
constexpr Param<bool>::Description kPublishEsdfDistanceSliceParamDesc{
  "publish_esdf_distance_slice", true,
  "Whether to output a distance slice of the ESDF to be used for path planning."};

constexpr Param<bool>::Description kUseColorParamDesc{
  "use_color", true,
  "Whether to integrate color images to color the mesh."};

constexpr Param<bool>::Description kUseDepthParamDesc{"use_depth", true,
  "Whether to integrate depth images."};

constexpr Param<bool>::Description kUseLidarParamDesc{
  "use_lidar", true, "Whether to integrate LiDAR scans."};

// ======= INPUT DATA PARAMS =======
constexpr Param<int>::Description kNumCamerasParamDesc{
  "num_cameras", 1,
  "Number of cameras supported (number of subscribers created)"};

constexpr Param<int>::Description kMaximumSensorMessageQueueLengthParamDesc{
  "maximum_sensor_message_queue_length", 10,
  "How many messages to store in the sensor messages queues (depth, color, lidar) before deleting "
  "oldest messages."};

// ======= MAPPING PARAMS =======
constexpr Param<float>::Description kVoxelSizeParamDesc{"voxel_size", .05F,
  "Voxel size (side of cube in meters) to use for the map."};

constexpr StringParam::Description kMappingTypeParamDesc{"mapping_type", "static_tsdf",
  "Type of mapper to use. See docs for description"};

constexpr Param<EsdfMode>::Description kEsdfModeParamDesc{"esdf_mode", static_cast<EsdfMode>(1),
  "Whether to compute the ESDF in 3D (0) or 2D (1)."};

constexpr Param<float>::Description kMapClearingRadiusMParamDesc{
  "map_clearing_radius_m", 5.F,
  "Radius around the ``map_clearing_frame_id`` outside which we clear the map. "
  "Note that values <= 0.0 indicate that no clearing is performed."};

constexpr StringParam::Description kMapClearingFrameIdParamDesc{
  "map_clearing_frame_id", "base_link", "The name of the TF frame around which we clear the map."};

// ======= LIDAR PARAMS =======
constexpr Param<int>::Description kLidarWidthParamDesc{"lidar_width", 1800,
  "Width of the LIDAR scan, in number of beams. Default works for the *VLP16*."};

constexpr Param<int>::Description kLidarHeightParamDesc{"lidar_height", 16,
  "Height of the LIDAR scan, in number of beams. Default works for the *VLP16*."};

constexpr Param<float>::Description kLidarVerticalFovRadParamDesc{
  "lidar_vertical_fov_rad", degreeToRadians(30.F),
  "The vertical field of view of the LIDAR scan, in radians (assuming beams are centered around 0 "
  " elevation). Default is for the *VLP16*."};

constexpr Param<bool>::Description kUseNonEqualVerticalFovLidarParamsParamDesc{
  "use_non_equal_vertical_fov_lidar_params", false,
  "Whether to use non equal vertical FoV for the LiDAR (not centered around 0 elevation). "
  "Should be set to false for a *VLP16* and to true for *Hesai PandarXT32*. The LiDAR model will "
  "use the ``lidar_vertical_fov_rad`` parameter if set to false and "
  "``min_angle_below_zero_elevation_rad`` / ``max_angle_below_zero_elevation_rad`` "
  " if set to true."};

constexpr Param<float>::Description kMinAngleBelowZeroElevationRadParamDesc{
  "min_angle_below_zero_elevation_rad", degreeToRadians(20.F),
  "The angle below zero elevation of the lowest beam (specified as a positive number in radians). "
  "Default is for the *Hesai PandarXT32*."};

constexpr Param<float>::Description kMaxAngleAboveZeroElevationRadParamDesc{
  "max_angle_above_zero_elevation_rad", degreeToRadians(15.F),
  "The angle above zero elevation of the highest beam (specified as a positive number in radians). "
  "Default is for the *Hesai PandarXT32*."};

// ======= VISUALIZATION PARAMS =======
constexpr StringParam::Description kSliceVisualizationAttachmentFrameIdParamDesc{
  "slice_visualization_attachment_frame_id", "base_link",
  "Frame to which the map slice bounds visualization is centered on the ``xy``-plane."};

constexpr Param<float>::Description kSliceVisualizationSideLengthParamDesc{
  "slice_visualization_side_length", 10.F,
  "Side length of the map slice bounds visualization plane."};

constexpr Param<float>::Description kLayerVisualizationMaxTsdfDistanceMParamDesc{
  "layer_visualization_max_tsdf_distance_m", .05F,
  "TSDF voxels with a distance above this value are not visualized."};

constexpr Param<float>::Description kLayerVisualizationMinTsdfWeightParamDesc{
  "layer_visualization_min_tsdf_weight", .1F,
  "TSDF voxels with a weight lower than this value are not visualized."};

constexpr Param<float>::Description kLayerVisualizationExclusionHeightMParamDesc{
  "layer_visualization_exclusion_height_m", 2.F,
  "Voxels with a z coordinate above this value are not visualized."};

constexpr Param<float>::Description kLayerVisualizationExclusionRadiusMParamDesc{
  "layer_visualization_exclusion_radius_m", 5.f,
  "Voxels further from the robot than this value are not visualized."};

constexpr Param<float>::Description kMaxBackProjectionDistanceParamDesc{
  "max_back_projection_distance", 5.F,
  "The maximum depth in meters when visualizing the back-projected point cloud."};

constexpr Param<bool>::Description kLayerVisualizationUndoGammaCorrectionParamDesc{
  "layer_visualization_undo_gamma_correction", false,
  "Apply a undo-gamma operation on the RGB values of the visualized layer. "
  "Usable if the rendering pipeline includes gamma correction, which is the case when rendering "
  "mesh markers in Foxglove."};

constexpr Param<int>::Description kBackProjectionSubsamplingParamDesc{
  "back_projection_subsampling", 1, "Only back project and publish every n-th depth image."};

constexpr Param<bool>::Description kEnableMeshMarkersParamDesc{
  "enable_mesh_markers", false,
  "Whether to also publish a mesh marker message in additional to the custom nvlbox mesh message."};

// ======= UPDATE RATES =======
constexpr Param<float>::Description kIntegrateDepthRateHzParamDesc{
  "integrate_depth_rate_hz", 40.F,
  "The desired integration frequency of depth images."};

constexpr Param<float>::Description kIntegrateColorRateHzParamDesc{
  "integrate_color_rate_hz", 5.F,
  "The desired integration frequency of color images."};

constexpr Param<float>::Description kIntegrateLidarRateHzParamDesc{
  "integrate_lidar_rate_hz", 40.F,
  "The desired integration frequency of lidar scans."};

constexpr Param<float>::Description kUpdateMeshRateHzParamDesc{
  "update_mesh_rate_hz", 5.F,
  "The desired mesh update frequency."};

constexpr Param<float>::Description kUpdateEsdfRateHzParamDesc{
  "update_esdf_rate_hz", 5.F,
  "The desired esdf update frequency."};

constexpr Param<float>::Description kPublishLayerRateHzParamDesc{
  "publish_layer_rate_hz", 10.F, "The desired rate for publishing layer visualization messages."};

constexpr Param<float>::Description kDecayTsdfRateHzParamDesc{"decay_tsdf_rate_hz", 5.F,
  "The desired rate for decaying the TSDF layer."};

constexpr Param<float>::Description kDecayDynamicOccupancyRateHzParamDesc{
  "decay_dynamic_occucancy_rate_hz", 10.F,
  "The desired rate for decaying the dynamic occupancy layer."};

constexpr Param<float>::Description kClearMapOutsideRadiusRateHzParamDesc{
  "clear_map_outside_radius_rate_hz", 1.F,
  "The desired rate for clearing the map from blocks far away from the robot."};

constexpr Param<int>::Description kTickPeriodMsParamDesc{
  "tick_period_ms", 10, "Specifies How often the main tick function of the node is ticked."};

constexpr Param<int>::Description kPrintStatisticsOnConsolePeriodMsParamDesc{
  "print_statistics_on_console_period_ms", 10000,
  "Specified how often to print timing and rate statistics to the terminal."};

constexpr Param<bool>::Description kPrintTimingsToConsoleParamDesc{"print_timings_to_console",
  false,
  "Whether to print timing stats to the console."};

constexpr Param<bool>::Description kPrintRatesToConsoleParamDesc{"print_rates_to_console", false,
  "Whether to print rate stats to the console."};

constexpr Param<bool>::Description kPrintDelaysToConsoleParamDesc{"print_delays_to_console", false,
  "Whether to print delay stats to the console."};

// ======= OUTPUT PARAMS =======
constexpr Param<float>::Description kEsdfAndGradientsUnobservedValueParamDesc{
  "esdf_and_gradients_unobserved_value", -1000.F,
  "This value will be used for for unobserved voxels in the dense output grid."};

/// Container for all node params
class NvbloxNodeParams
{
public:
  NvbloxNodeParams() = default;

  StringParam global_frame{kGlobalFrameParamDesc};
  StringParam pose_frame{kPoseFrameParamDesc};
  StringParam map_clearing_frame_id{kMapClearingFrameIdParamDesc};
  StringParam slice_visualization_attachment_frame_id{
    kSliceVisualizationAttachmentFrameIdParamDesc};

  Param<bool> print_timings_to_console{kPrintTimingsToConsoleParamDesc};
  Param<bool> print_rates_to_console{kPrintRatesToConsoleParamDesc};
  Param<bool> print_delays_to_console{kPrintDelaysToConsoleParamDesc};
  Param<bool> enable_mesh_markers{kEnableMeshMarkersParamDesc};
  Param<bool> use_non_equal_vertical_fov_lidar_params{kUseNonEqualVerticalFovLidarParamsParamDesc};
  Param<bool> publish_esdf_distance_slice{kPublishEsdfDistanceSliceParamDesc};
  Param<bool> use_color{kUseColorParamDesc};
  Param<bool> use_depth{kUseDepthParamDesc};
  Param<bool> use_lidar{kUseLidarParamDesc};
  Param<bool> layer_visualization_undo_gamma_correction{
    kLayerVisualizationUndoGammaCorrectionParamDesc};

  Param<int> maximum_sensor_message_queue_length{kMaximumSensorMessageQueueLengthParamDesc};
  Param<int> back_projection_subsampling{kBackProjectionSubsamplingParamDesc};
  Param<int> tick_period_ms{kTickPeriodMsParamDesc};
  Param<int> print_statistics_on_console_period_ms{kPrintStatisticsOnConsolePeriodMsParamDesc};
  Param<int> num_cameras{kNumCamerasParamDesc};
  Param<int> lidar_width{kLidarWidthParamDesc};
  Param<int> lidar_height{kLidarHeightParamDesc};

  Param<float> lidar_vertical_fov_rad{kLidarVerticalFovRadParamDesc};
  Param<float> voxel_size{kVoxelSizeParamDesc};
  Param<float> min_angle_below_zero_elevation_rad{kMinAngleBelowZeroElevationRadParamDesc};
  Param<float> max_angle_above_zero_elevation_rad{kMaxAngleAboveZeroElevationRadParamDesc};
  Param<float> slice_visualization_side_length{kSliceVisualizationSideLengthParamDesc};
  Param<float> layer_visualization_max_tsdf_distance_m{
    kLayerVisualizationMaxTsdfDistanceMParamDesc};
  Param<float> layer_visualization_min_tsdf_weight{kLayerVisualizationMinTsdfWeightParamDesc};
  Param<float> layer_visualization_exclusion_height_m{kLayerVisualizationExclusionHeightMParamDesc};
  Param<float> layer_visualization_exclusion_radius_m{kLayerVisualizationExclusionRadiusMParamDesc};
  Param<float> integrate_depth_rate_hz{kIntegrateDepthRateHzParamDesc};
  Param<float> integrate_color_rate_hz{kIntegrateColorRateHzParamDesc};
  Param<float> integrate_lidar_rate_hz{kIntegrateLidarRateHzParamDesc};
  Param<float> update_mesh_rate_hz{kUpdateMeshRateHzParamDesc};
  Param<float> update_esdf_rate_hz{kUpdateEsdfRateHzParamDesc};
  Param<float> publish_layer_rate_hz{kPublishLayerRateHzParamDesc};
  Param<float> decay_tsdf_rate_hz{kDecayTsdfRateHzParamDesc};
  Param<float> decay_dynamic_occupancy_rate_hz{kDecayDynamicOccupancyRateHzParamDesc};
  Param<float> clear_map_outside_radius_rate_hz{kClearMapOutsideRadiusRateHzParamDesc};
  Param<float> esdf_and_gradients_unobserved_value{kEsdfAndGradientsUnobservedValueParamDesc};
  Param<float> map_clearing_radius_m{kMapClearingRadiusMParamDesc};
  Param<float> max_back_projection_distance{kMaxBackProjectionDistanceParamDesc};

  Param<EsdfMode> esdf_mode{kEsdfModeParamDesc};
  // TODO(dtingdahl) handle enum-from-string logic more elegant so we don't need a separate member
  // for the enum mapping type
  StringParam mapping_type_str{kMappingTypeParamDesc};
  MappingType mapping_type;
};

/// Register all parameter with ROS and initialize them if set from ROS.
void initializeRosParams(
  rclcpp::Node * node, NvbloxNodeParams * params,
  std::optional<nvblox::parameters::ParameterTreeNode *> parameter_tree = std::nullopt);

}  // namespace nvblox


#endif  // NVBLOX_ROS__NVBLOX_NODE_PARAMS_HPP_
