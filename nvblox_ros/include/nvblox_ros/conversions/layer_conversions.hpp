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

#ifndef NVBLOX_ROS__CONVERSIONS__LAYER_CONVERSIONS_HPP_
#define NVBLOX_ROS__CONVERSIONS__LAYER_CONVERSIONS_HPP_

#include <nvblox/nvblox.h>

#include <memory>

#include "nvblox_ros/conversions/pointcloud_conversions.hpp"
#include "nvblox/serialization/layer_serializer_gpu.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace nvblox
{
namespace conversions
{

/// Conversions from different layer types to Marker message for visualization
class LayerConverter
{
public:
  LayerConverter() = default;

  /// Convert a color layer to a marker messag with RGB colors.
  /// A tsdf layer is used to determine which voxels to convert.
  ///
  /// @param tsdf_layer           TSDF layer.
  /// @param color_layer          Color layer.
  /// @param undo_gamma_correction  If true, undo sRGB gamma correction for RGB values.
  /// @param max_tsdf_distance    Max tsdf distance for voxels.
  /// @param min_tsdf_weight      Min tsdf weight for voxels.
  /// @param exclusion_height_m   Exclude blocks with a Z coordnate larger than this.
  /// @param exclusion_radius_m   Exclude blocks outside this radius.
  /// @param exclusion_center_m   Center for radial exclusion.
  /// @param marker_msg           Resulting marker message.
  /// @param cuda_stream          Cuda strewm for serializing layers.
  void markerMsgFromColorLayer(
    const TsdfLayer & tsdf_layer,
    const ColorLayer & color_layer,
    const bool undo_gamma_correction,
    const float max_tsdf_distance,
    const float min_tsdf_weight,
    const float exclusion_height_m,
    const float exclusion_radius_m,
    const Vector3f & exclusion_center,
    visualization_msgs::msg::Marker * marker_msg,
    const CudaStream & cuda_stream);

  /// Convert a color layer to a marker message with RGB colors.
  /// Am esdf layer is used to determine which voxels to convert.
  ///
  /// @param esdf_layer             ESDF layer.
  /// @param color_layer            Color layer.
  /// @param undo_gamma_correction  If true, undo sRGB gamma correction for RGB values.
  /// @param exclusion_height_m     Exclude blocks with a Z coordnate larger than this.
  /// @param exclusion_radius_m     Exclude blocks outside this radius.
  /// @param exclusion_center_m     Center for radial exclusion.
  /// @param marker_msg             Resulting marker message.
  /// @param cuda_stream            Cuda strewm for serializing layers.
  void markerMsgFromColorLayer(
    const EsdfLayer & esdf_layer,
    const ColorLayer & color_layer,
    const bool undo_gamma_correction,
    const float exclusion_height_m,
    const float exclusion_radius_m,
    const Vector3f & exclusion_center,
    visualization_msgs::msg::Marker * marker_msg,
    const CudaStream & cuda_stream);

  /// Convert an occupancy layer to a marker message. Intensity of the points is set to
  /// occupancy probability.
  ///
  /// @param occupancy_layer      Occupancy layer.
  /// @param exclusion_height_m   Exclude blocks with a Z coordnate larger than this.
  /// @param exclusion_radius_m   Exclude blocks outside this radius.
  /// @param exclusion_center_m   Center for radial exclusion.
  /// @param marker_msg           Resulting marker message.
  /// @param cuda_stream          Cuda strewm for serializing layers.
  void markerMsgFromOccupancyLayer(
    const OccupancyLayer & occupancy_layer,
    const float exclusion_height_m,
    const float exclusion_radius_m,
    const Vector3f & exclusion_center,
    visualization_msgs::msg::Marker * marker_msg,
    const CudaStream & cuda_stream);

  /// Convert an occupancy layer to a marker message. Intensity of the points is set to
  /// tsdf weight.
  ///
  /// @param tsdf_layer           TSDF layer.
  /// @param max_tsdf_distance    Max tsdf distance for voxels.
  /// @param min_tsdf_weight      Min tsdf weight for voxels.
  /// @param exclusion_height_m   Exclude blocks with a Z coordnate larger than this.
  /// @param exclusion_radius_m   Exclude blocks outside this radius.
  /// @param exclusion_center_m   Center for radial exclusion.
  /// @param marker_msg           Resulting marker message.
  /// @param cuda_stream          Cuda strewm for serializing layers.
  void markerMsgFromTsdfLayer(
    const TsdfLayer & tsdf_layer,
    const float max_tsdf_distance,
    const float min_tsdf_weight,
    const float exclusion_height_m,
    const float exclusion_radius_m,
    const Vector3f & exclusion_center,
    visualization_msgs::msg::Marker * marker_msg,
    const CudaStream & cuda_stream);

  /// Convert a freespace layer to a marker message. Intensity of the points is set to
  /// consecutive occupancy duration. A tsdf layer is used to determine which voxels to convert.
  ///
  /// @param tsdf_layer           TSDF layer.
  /// @param freespace_layer      Freespace layer.
  /// @param max_tsdf_distance    Max tsdf distance for voxels.
  /// @param min_tsdf_weight      Min tsdf weight for voxels.
  /// @param exclusion_height_m   Exclude blocks with a Z coordnate larger than this.
  /// @param exclusion_radius_m   Exclude blocks outside this radius.
  /// @param exclusion_center_m   Center for radial exclusion.
  /// @param marker_msg           Resulting marker message.
  /// @param cuda_stream          Cuda strewm for serializing layers.
  void markerMsgFromFreespaceLayer(
    const TsdfLayer & tsdf_layer,
    const FreespaceLayer & freespace_layer,
    const float max_tsdf_distance,
    const float min_tsdf_weight,
    const float exclusion_height_m,
    const float exclusion_radius_m,
    const Vector3f & exclusion_center,
    visualization_msgs::msg::Marker * marker_msg,
    const CudaStream & cuda_stream);

private:
  ColorLayerSerializerGpu color_layer_serializer_;
  OccupancyLayerSerializerGpu occupancy_layer_serializer_;
  TsdfLayerSerializerGpu tsdf_layer_serializer_;
  EsdfLayerSerializerGpu esdf_layer_serializer_;
  FreespaceLayerSerializerGpu freespace_layer_serializer_;
};

}   // namespace conversions
}  // namespace nvblox

#endif  // NVBLOX_ROS__CONVERSIONS__LAYER_CONVERSIONS_HPP_
