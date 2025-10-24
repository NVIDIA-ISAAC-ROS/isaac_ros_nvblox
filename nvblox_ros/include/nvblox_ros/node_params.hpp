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

#ifndef NVBLOX_ROS__NODE_PARAMS_HPP_
#define NVBLOX_ROS__NODE_PARAMS_HPP_

#include "nvblox/utils/params.h"
#include "nvblox/mapper/multi_mapper.h"
#include "nvblox_ros/dataset_types.hpp"

#include <rclcpp/rclcpp.hpp>

namespace nvblox
{

/// Convert degrees to radians
constexpr float degreeToRadians(float degrees)
{
  return degrees * M_PI / 180.F;
}

// ======= CUDA CONFIG PARAMS =======
constexpr Param<CudaStreamType>::Description kCudaStreamTypeParamDesc{"cuda_stream_type",
  static_cast<CudaStreamType>(1),
  "The cuda stream type nvblox ros node will create. Default option is a blocking async "
  "stream(1), we can also use cuda default stream(0) or non blocking async stream(2) or "
  "per thread default stream(3)"};

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

constexpr Param<bool>::Description kUseSegmentationParamDesc{"use_segmentation", false,
  "Whether to integrate segmentation masks."};

constexpr Param<bool>::Description kUseLidarParamDesc{
  "use_lidar", true, "Whether to integrate LiDAR scans."};

// ======= INPUT DATA PARAMS =======
constexpr Param<int>::Description kNumCamerasParamDesc{
  "num_cameras", 1,
  "Number of cameras supported (number of subscribers created)"};

constexpr Param<int>::Description kMaximumSensorMessageQueueLengthParamDesc{
  "maximum_input_queue_length", 10,
  "How many items to store in the input queues (depth, color, lidar, services) before deleting "
  "oldest items."};

// ======= MAPPING PARAMS =======
constexpr Param<float>::Description kVoxelSizeParamDesc{"voxel_size", .05F,
  "Voxel size (side of cube in meters) to use for the map."};

constexpr StringParam::Description kMappingTypeParamDesc{"mapping_type", "static_tsdf",
  "Type of mapper to use. See docs for description"};

constexpr StringParam::Description kEsdfModeParamDesc{"esdf_mode", "2d",
  "Whether to compute the ESDF in 2d or 3d."};

constexpr Param<float>::Description kMapClearingRadiusMParamDesc{
  "map_clearing_radius_m", 5.F,
  "Radius around the ``map_clearing_frame_id`` outside which we clear the map. "
  "Note that values <= 0.0 indicate that no clearing is performed."};

constexpr StringParam::Description kMapClearingFrameIdParamDesc{
  "map_clearing_frame_id", "base_link", "The name of the TF frame around which we clear the map."};

constexpr StringParam::Description kAfterShutdownMapSavePathParamDesc{
  "after_shutdown_map_save_path", "", "The path used to save the map after shutdown."};

constexpr Param<float>::Description kDistanceMapUnknownValueOptimistic{
  "distance_map_unknown_value_optimistic", 1000.0F,
  "The distance inside the distance map for unknown value."};

constexpr Param<float>::Description kDistanceMapUnknownValuePessimistic{
  "distance_map_unknown_value_pessimistic", -1000.0F,
  "The distance inside the distance map for unknown value."};

// ======= LIDAR PARAMS =======
constexpr Param<int>::Description kLidarWidthParamDesc{"lidar_width", 1800,
  "Width of the LIDAR scan, in number of beams. Default works for the *VLP16*."};

constexpr Param<int>::Description kLidarHeightParamDesc{"lidar_height", 16,
  "Height of the LIDAR scan, in number of beams. Default works for the *VLP16*."};

constexpr Param<float>::Description kLidarVerticalFovRadParamDesc{
  "lidar_vertical_fov_rad", degreeToRadians(30.F),
  "The vertical field of view of the LIDAR scan, in radians (assuming beams are centered around 0 "
  " elevation). Default is for the *VLP16*."};

constexpr Param<float>::Description kLidarMinRangeM{
  "lidar_min_valid_range_m", 0.1F,
  "The minimum valid range of the lidar."};

constexpr Param<float>::Description kLidarMaxRangeM{
  "lidar_max_valid_range_m", 50.0F,
  "The maximum valid range of the lidar."};

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
constexpr StringParam::Description kEsdfSliceBoundsVisualizationAttachmentFrameIdParamDesc{
  "esdf_slice_bounds_visualization_attachment_frame_id", "base_link",
  "Frame to which the esdf slice bounds visualization is centered on the ``xy``-plane."};

constexpr Param<float>::Description kEsdfSliceBoundsVisualizationSideLengthParamDesc{
  "esdf_slice_bounds_visualization_side_length", 10.F,
  "Side length of the esdf slice bounds visualization plane."};

constexpr StringParam::Description kGroundPlaneVisualizationAttachmentFrameIdParamDesc{
  "ground_plane_visualization_attachment_frame_id", "base_link",
  "Frame to which the ground plane visualization is centered on."};

constexpr Param<float>::Description kGroundPlaneVisualizationSideLengthParamDesc{
  "ground_plane_visualization_side_length", 10.F,
  "Side length of the ground plane visualization plane."};

constexpr StringParam::Description kWorkspaceHeightBoundsVisualizationAttachmentFrameIdParamDesc{
  "workspace_height_bounds_visualization_attachment_frame_id", "base_link",
  "Frame to which the workspace height bounds visualization is centered on the ``xy``-plane."};

constexpr Param<float>::Description kWorkspaceHeightBoundsVisualizationSideLengthParamDesc{
  "workspace_height_bounds_visualization_side_length", 10.F,
  "Side length of the workspace height bounds visualization plane."};

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

constexpr Param<float>::Description kLayerStreamerBandwidthLimitMbpsParamDesc{
  "layer_streamer_bandwidth_limit_mbps", 30.f,
  "Bandwidth limit for streaming layer visualizations (over WiFi) in mega-bits per second."};

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

constexpr Param<float>::Description kPublishDebugVisRateHzParamDesc{
  "publish_debug_vis_rate_hz", 2.F,
  "The desired rate for publishing debug visualization messages."};

constexpr Param<float>::Description kDecayTsdfRateHzParamDesc{"decay_tsdf_rate_hz", 5.F,
  "The desired rate for decaying the TSDF layer."};

constexpr Param<float>::Description kDecayDynamicOccupancyRateHzParamDesc{
  "decay_dynamic_occupancy_rate_hz", 10.F,
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

constexpr Param<bool>::Description kPrintQueueDropsToConsoleParamDesc{
  "print_queue_drops_to_console", false,
  "Whether to print message drops from queues to the console."};

// ======= OUTPUT PARAMS =======
constexpr Param<float>::Description kEsdfAndGradientsUnobservedValueParamDesc{
  "esdf_and_gradients_unobserved_value", -1000.F,
  "This value will be used for for unobserved voxels in the dense output grid."};
constexpr Param<bool>::Description kOutputPessimisticDistanceMap{
  "output_pessimistic_distance_map", true,
  "Whether or not to output the pessimistic distance map."};

// ======= FUSER NODE PARAMS =======
constexpr StringParam::Description kDatasetTypeDesc{"dataset_type", "3dmatch",
  "Which type of dataset to load: 3dmatch, replica or redwood."};
constexpr StringParam::Description kDatasetPath{
  "dataset_path", "", "Path to dataset to load."};
constexpr Param<int>::Description kNumberOfFramesToIntegrate{
  "number_of_frames_to_integrate", -1, "Number of frames to process. Negative means process all."};
constexpr Param<bool>::Description kUpdateOnKey{"update_on_key", false,
  "Whether to fuse the dataset full speed or update on key inputs from terminal."};
constexpr StringParam::Description kFuserDepthTopic{"fuser_depth_topic",
  "/front_stereo_camera/depth", "Depth topic to read from a ROSbag and fuse in the FuserNode."};
constexpr StringParam::Description kFuserDepthCameraInfoTopic{"fuser_depth_camera_info_topic",
  "/front_stereo_camera/camera_info",
  "Depth camera_info topic to read from a ROSbag and fuse in the FuserNode."};
constexpr StringParam::Description kFuserColorTopic{"fuser_color_topic",
  "/front_stereo_camera/left/image_rect",
  "Color topic to read from a ROSbag and fuse in the FuserNode."};
constexpr StringParam::Description kFuserColorCameraInfoTopic{"fuser_color_camera_info_topic",
  "/front_stereo_camera/left/camera_info_rect",
  "Color camera_info topic to read from a ROSbag and fuse in the FuserNode."};
constexpr Param<float>::Description kTfLeadTimeS{"tf_lead_time_s", 0.5f,
  "Pre-load tf transforms the amount of time ahead of the image topics in the FuserNode."};

/// Container for all node params that are shared among nodes (fuser and nvblox node).
class BaseNodeParams
{
public:
  BaseNodeParams() = default;
  virtual ~BaseNodeParams() = default;

  Param<CudaStreamType> cuda_stream_type{kCudaStreamTypeParamDesc};
  StringParam global_frame{kGlobalFrameParamDesc};
  Param<float> voxel_size{kVoxelSizeParamDesc};
  Param<float> distance_map_unknown_value_pessimistic{kDistanceMapUnknownValuePessimistic};
  Param<float> distance_map_unknown_value_optimistic{kDistanceMapUnknownValueOptimistic};
  Param<float> max_back_projection_distance{kMaxBackProjectionDistanceParamDesc};
  Param<float> layer_visualization_min_tsdf_weight{kLayerVisualizationMinTsdfWeightParamDesc};
  Param<float> layer_visualization_exclusion_height_m{kLayerVisualizationExclusionHeightMParamDesc};
  Param<float> layer_visualization_exclusion_radius_m{kLayerVisualizationExclusionRadiusMParamDesc};
  Param<float> layer_streamer_bandwidth_limit_mbps{kLayerStreamerBandwidthLimitMbpsParamDesc};

  // TODO(dtingdahl) handle enum-from-string logic more elegant so we don't need a separate member
  // for the enum mapping type
  StringParam esdf_mode_str{kEsdfModeParamDesc};
  EsdfMode esdf_mode;
  StringParam mapping_type_str{kMappingTypeParamDesc};
  MappingType mapping_type;
};

/// Container for all node params of the nvblox node.
class NvbloxNodeParams : public BaseNodeParams
{
public:
  NvbloxNodeParams() = default;

  Param<CudaStreamType> cuda_stream_type{kCudaStreamTypeParamDesc};
  StringParam pose_frame{kPoseFrameParamDesc};
  StringParam map_clearing_frame_id{kMapClearingFrameIdParamDesc};
  StringParam after_shutdown_map_save_path{kAfterShutdownMapSavePathParamDesc};
  StringParam esdf_slice_bounds_visualization_attachment_frame_id{
    kEsdfSliceBoundsVisualizationAttachmentFrameIdParamDesc};
  StringParam workspace_height_bounds_visualization_attachment_frame_id{
    kWorkspaceHeightBoundsVisualizationAttachmentFrameIdParamDesc};

  StringParam ground_plane_visualization_attachment_frame_id{
    kGroundPlaneVisualizationAttachmentFrameIdParamDesc};
  Param<bool> print_timings_to_console{kPrintTimingsToConsoleParamDesc};
  Param<bool> print_rates_to_console{kPrintRatesToConsoleParamDesc};
  Param<bool> print_delays_to_console{kPrintDelaysToConsoleParamDesc};
  Param<bool> print_queue_drops_to_console{kPrintQueueDropsToConsoleParamDesc};
  Param<bool> use_non_equal_vertical_fov_lidar_params{kUseNonEqualVerticalFovLidarParamsParamDesc};
  Param<bool> publish_esdf_distance_slice{kPublishEsdfDistanceSliceParamDesc};
  Param<bool> use_color{kUseColorParamDesc};
  Param<bool> use_depth{kUseDepthParamDesc};
  Param<bool> use_segmentation{kUseSegmentationParamDesc};
  Param<bool> use_lidar{kUseLidarParamDesc};
  Param<bool> layer_visualization_undo_gamma_correction{
    kLayerVisualizationUndoGammaCorrectionParamDesc};
  Param<bool> output_pessimistic_distance_map{kOutputPessimisticDistanceMap};

  Param<int> maximum_input_queue_length{kMaximumSensorMessageQueueLengthParamDesc};
  Param<int> back_projection_subsampling{kBackProjectionSubsamplingParamDesc};
  Param<int> tick_period_ms{kTickPeriodMsParamDesc};
  Param<int> print_statistics_on_console_period_ms{kPrintStatisticsOnConsolePeriodMsParamDesc};
  Param<int> num_cameras{kNumCamerasParamDesc};
  Param<int> lidar_width{kLidarWidthParamDesc};
  Param<int> lidar_height{kLidarHeightParamDesc};

  Param<float> lidar_vertical_fov_rad{kLidarVerticalFovRadParamDesc};
  Param<float> lidar_min_valid_range_m{kLidarMinRangeM};
  Param<float> lidar_max_valid_range_m{kLidarMaxRangeM};
  Param<float> min_angle_below_zero_elevation_rad{kMinAngleBelowZeroElevationRadParamDesc};
  Param<float> max_angle_above_zero_elevation_rad{kMaxAngleAboveZeroElevationRadParamDesc};
  Param<float> esdf_slice_bounds_visualization_side_length{
    kEsdfSliceBoundsVisualizationSideLengthParamDesc};
  Param<float> workspace_height_bounds_visualization_side_length{
    kWorkspaceHeightBoundsVisualizationSideLengthParamDesc};
  Param<float> ground_plane_visualization_side_length{
    kGroundPlaneVisualizationSideLengthParamDesc};
  Param<float> integrate_depth_rate_hz{kIntegrateDepthRateHzParamDesc};
  Param<float> integrate_color_rate_hz{kIntegrateColorRateHzParamDesc};
  Param<float> integrate_lidar_rate_hz{kIntegrateLidarRateHzParamDesc};
  Param<float> update_mesh_rate_hz{kUpdateMeshRateHzParamDesc};
  Param<float> update_esdf_rate_hz{kUpdateEsdfRateHzParamDesc};
  Param<float> publish_layer_rate_hz{kPublishLayerRateHzParamDesc};
  Param<float> publish_debug_vis_rate_hz{kPublishDebugVisRateHzParamDesc};
  Param<float> decay_tsdf_rate_hz{kDecayTsdfRateHzParamDesc};
  Param<float> decay_dynamic_occupancy_rate_hz{kDecayDynamicOccupancyRateHzParamDesc};
  Param<float> clear_map_outside_radius_rate_hz{kClearMapOutsideRadiusRateHzParamDesc};
  Param<float> esdf_and_gradients_unobserved_value{kEsdfAndGradientsUnobservedValueParamDesc};
  Param<float> map_clearing_radius_m{kMapClearingRadiusMParamDesc};
};

/// Container for all node params of the fuser node.
class FuserNodeParams : public BaseNodeParams
{
public:
  FuserNodeParams() = default;
  StringParam dataset_path{kDatasetPath};
  Param<int> number_of_frames_to_integrate{kNumberOfFramesToIntegrate};
  Param<bool> update_on_key{kUpdateOnKey};

  // TODO(dtingdahl) handle enum-from-string logic more elegant so we don't need a separate member
  // for the enum mapping type
  StringParam dataset_type_str{kDatasetTypeDesc};
  RosDatasetType dataset_type;

  // Params for ROSbag loading
  StringParam depth_topic{kFuserDepthTopic};
  StringParam depth_camera_info_topic{kFuserDepthCameraInfoTopic};
  StringParam color_topic{kFuserColorTopic};
  StringParam color_camera_info_topic{kFuserColorCameraInfoTopic};
  Param<float> tf_lead_time_s{kTfLeadTimeS};
};

/// Register all parameters that are shared between ROS nodes and initialize them if set from ROS.
void initializeBaseNodeParams(
  rclcpp::Node * node, BaseNodeParams * params,
  std::optional<nvblox::parameters::ParameterTreeNode *> parameter_tree = std::nullopt);

/// Register all nvblox node parameters and initialize them if set from ROS.
void initializeNvbloxNodeParams(
  rclcpp::Node * node, NvbloxNodeParams * params,
  std::optional<nvblox::parameters::ParameterTreeNode *> parameter_tree = std::nullopt);

/// Register all fuser node parameters with ROS and initialize them if set from ROS.
void initializeFuserNodeParams(
  rclcpp::Node * node, FuserNodeParams * params,
  std::optional<nvblox::parameters::ParameterTreeNode *> parameter_tree = std::nullopt);

}  // namespace nvblox


#endif  // NVBLOX_ROS__NODE_PARAMS_HPP_
