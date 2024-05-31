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

#include "nvblox_ros/conversions/layer_conversions.hpp"
#include <rclcpp/rclcpp.hpp>

namespace nvblox
{
namespace conversions
{

template<typename T> T clamp(const T value, const T minval, const T maxval)
{
  return std::max(minval, std::min(maxval, value));
}

std_msgs::msg::ColorRGBA rgbFromScalar(const float v)
{
  std_msgs::msg::ColorRGBA color;
  color.r = v;
  color.g = v;
  color.b = v;
  color.a = 1.F;

  return color;
}

// Functor for creating an RGB message out of a color voxel.
//
// An optional undoing of gamma correction can be performed to prevent washed-out colors for
// rendering pipelines that already does gamma correction (e.g. Foxglove).
class ColorVoxelToRgb
{
public:
  std_msgs::msg::ColorRGBA operator()(const ColorVoxel & voxel)
  {
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    if (undo_gamma_correction_) {
      color.r = undo_gamma_correction_lut_[voxel.color.r];
      color.g = undo_gamma_correction_lut_[voxel.color.g];
      color.b = undo_gamma_correction_lut_[voxel.color.b];
    } else {
      color.r = static_cast<float>(voxel.color.r) / 255.F;
      color.g = static_cast<float>(voxel.color.g) / 255.F;
      color.b = static_cast<float>(voxel.color.b) / 255.F;
    }
    return color;
  }

  explicit ColorVoxelToRgb(const bool undo_gamma_correction = false)
  : undo_gamma_correction_(undo_gamma_correction)
  {
    buildLookupTable();
  }

  void buildLookupTable()
  {
    // Create lookup table for undoing gamma correction
    for (size_t i = 0; i < undo_gamma_correction_lut_.size(); ++i) {
      float val = static_cast<float>(i) / 255.F;

      if (val < 0.04045) {
        val /= 12.92F;
      } else {
        val = std::pow((val + 0.055) / 1.055, 2.4);
      }
      undo_gamma_correction_lut_[i] = std::max(0.F, std::min(1.F, val));
    }
  }

private:
  static std::array<float, 256> undo_gamma_correction_lut_;
  bool undo_gamma_correction_ = true;
};

std::array<float, 256> ColorVoxelToRgb::undo_gamma_correction_lut_;

// Convert a voxel into an rgb consumable by PointCloud2.
std_msgs::msg::ColorRGBA tsdfVoxelToRgb(const TsdfVoxel & voxel)
{
  constexpr float kMaxWeight = 5.F;
  const float v = clamp(voxel.weight / kMaxWeight, 0.F, 1.F);

  return rgbFromScalar(v);
}

// Convert a voxel into an rgb consumable by PointCloud2.
std_msgs::msg::ColorRGBA occupancyVoxelToRgb(const OccupancyVoxel &)
{
  // Occupancy voxels are always colored red.
  std_msgs::msg::ColorRGBA color;
  color.r = 1.F;
  color.g = 0.F;
  color.b = 0.F;
  color.a = 1.F;
  return color;
}

// Convert a voxel into an rgb consumable by PointCloud2.
std_msgs::msg::ColorRGBA freespaceVoxelToRgb(const FreespaceVoxel & voxel)
{
  constexpr float kMaxDurationMs = 5000.F;
  const int64_t duration = static_cast<int64_t>(voxel.consecutive_occupancy_duration_ms);

  return rgbFromScalar(std::min(static_cast<float>(duration) / kMaxDurationMs, 1.0F));
}

// Functor that returns true for occupancy voxels with an log_odds above a given threshold.
class OccupancyVoxelFilter
{
public:
  explicit OccupancyVoxelFilter(const float min_log_odds = 1E-3F)
  : min_log_odds_(min_log_odds) {}

  bool operator()(const OccupancyVoxel & voxel) {return voxel.log_odds > min_log_odds_;}

private:
  float min_log_odds_;
};

// Functor that returns true for freespance voxels with high confidence.
class FreespaceVoxelFilter
{
public:
  FreespaceVoxelFilter() = default;
  bool operator()(const FreespaceVoxel & voxel) {return voxel.is_high_confidence_freespace;}
};

// Functor that returns true for tsdf voxels within specified distance and weights thresholds.
class TsdfVoxelFilter
{
public:
  TsdfVoxelFilter(const float max_tsdf_distance, const float min_tsdf_weight)
  : max_tsdf_distance_(max_tsdf_distance), min_tsdf_weight_(min_tsdf_weight) {}

  bool operator()(const TsdfVoxel & voxel)
  {
    return (voxel.weight > min_tsdf_weight_) &&
           (voxel.distance < max_tsdf_distance_ && voxel.distance > -max_tsdf_distance_);
  }

private:
  float max_tsdf_distance_;
  float min_tsdf_weight_;
};

// Functor that returns true for esdf voxels on the surface
class EsdfVoxelFilter
{
public:
  EsdfVoxelFilter() = default;

  bool operator()(const EsdfVoxel & voxel) {return voxel.is_site;}
};

// Functor that returns true for blocks within an exclusion radius and below a given exclusion
// height.
class BlockFilter
{
public:
  BlockFilter(
    const float exclusion_height_m, const float exclusion_radius_m,
    const float block_size, const Vector3f & exclusion_center)
  : exclusion_height_m_(exclusion_height_m),
    exclusion_radius_sqm_(exclusion_radius_m * exclusion_radius_m), block_size_(block_size),
    exclusion_center_(exclusion_center) {}

  bool operator()(const Index3D & block_index)
  {
    const Vector3f block_position = getCenterPositionFromBlockIndex(block_size_, block_index);
    if (block_position.z() > exclusion_height_m_) {
      return false;
    } else {
      const float dist_sq = (exclusion_center_ - block_position).squaredNorm();
      return dist_sq < exclusion_radius_sqm_;
    }
  }

private:
  float exclusion_height_m_;
  float exclusion_radius_sqm_;
  float block_size_;
  Vector3f exclusion_center_;
};

// Convert layer to cubelist marker message for visualization
//
// The function accepts two layers.
//   * Layer1 is used to filter out voxels we don't want to visualize
//   * Layer2 is used to obtain color for the voxels
//
// This is useful if we e.g. want to visualize the color layer but use the TSDF layer to remove
// voxels with large distances. Note that is possible give the layer for both layer1 and layer2.
//
// @param serialized_layer1         Layer used for filtering unwanted voxels
// @param serialized_layer2         Layer used for visualizing color
// @param voxel_size                Size of visualized voxels
// @param voxel_in_layer1_valid     Functor that returns true if a voxel in layer1 should be kept
// @param voxel_in_layer2_to_color  Functor that converts a voxel in layer2 to a color
// @param block_size                Block size of visualized layer
// @param marker_msg                Resulting pointcloud message
template<typename LayerType1, typename LayerType2>
void markerMsgFromLayer(
  const std::shared_ptr<const SerializedLayer<typename LayerType1::VoxelType>> & serialized_layer1,
  const std::shared_ptr<const SerializedLayer<typename LayerType2::VoxelType>> & serialized_layer2,
  float voxel_size,
  const std::function<bool(typename LayerType1::VoxelType)> voxel_in_layer1_valid,
  const std::function<std_msgs::msg::ColorRGBA(typename LayerType2::VoxelType)>
  voxel_in_layer2_to_color,
  const float block_size, visualization_msgs::msg::Marker * marker_msg)
{
  CHECK_NOTNULL(marker_msg);

  const std::vector<Index3D> & block_indices = serialized_layer1->block_indices;

  constexpr int kVoxelsPerSide = LayerType1::VoxelBlockType::kVoxelsPerSide;
  static_assert(
    kVoxelsPerSide == LayerType2::VoxelBlockType::kVoxelsPerSide,
    "Layers must have same block dimension");

  const int max_num_points =
    block_indices.size() * kVoxelsPerSide * kVoxelsPerSide * kVoxelsPerSide;

  // All voxel are placed in the same marker message. This means that all voxels will be updated
  // every time we publish
  marker_msg->points.reserve(max_num_points);
  marker_msg->ns = "layer";
  marker_msg->id = 0;
  marker_msg->type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker_msg->action = visualization_msgs::msg::Marker::ADD;
  marker_msg->lifetime = rclcpp::Duration(0, 0);  // never decay. only update/add
  marker_msg->frame_locked = false;

  // Subtract a small amount from the scale to prevent clipping when rendering
  const float scale = voxel_size - 1E-3;
  marker_msg->scale.x = scale;
  marker_msg->scale.y = scale;
  marker_msg->scale.z = scale;

  // Need same number of blocks in both layers.
  CHECK(serialized_layer1->block_indices.size() == serialized_layer2->block_indices.size());

  // Go over all blocks
  for (size_t i = 0; i < block_indices.size(); ++i) {
    const Index3D & block_index = block_indices[i];

    // Figure out current block's offset in the serialized vectors
    const int offset_layer1 = serialized_layer1->block_offsets[i];
    const int offset_layer2 = serialized_layer2->block_offsets[i];
    const int num_layer1 = serialized_layer1->block_offsets[i + 1] - offset_layer1;
    const int num_layer2 = serialized_layer2->block_offsets[i + 1] - offset_layer2;
    if (num_layer1 != num_layer2) {
      continue;
    }

    // For each voxel in the block...
    for (int x = 0; x < kVoxelsPerSide; ++x) {
      for (int y = 0; y < kVoxelsPerSide; ++y) {
        for (int z = 0; z < kVoxelsPerSide; ++z) {
          const int lin_index = z + 8 * y + 64 * x;
          const typename LayerType1::VoxelType & layer1_voxel =
            serialized_layer1->voxels[offset_layer1 + lin_index];
          const typename LayerType2::VoxelType & layer2_voxel =
            serialized_layer2->voxels[offset_layer2 + lin_index];

          // Reject voxels using the provided functor
          if (!voxel_in_layer1_valid(layer1_voxel)) {
            continue;
          }

          // Get position of this voxel fron the indices
          const Vector3f pos =
            getCenterPositionFromBlockIndexAndVoxelIndex(block_size, block_index, {x, y, z});

          geometry_msgs::msg::Point point_msg;
          point_msg.x = pos(0);
          point_msg.y = pos(1);
          point_msg.z = pos(2);
          marker_msg->points.emplace_back(point_msg);

          marker_msg->colors.emplace_back(voxel_in_layer2_to_color(layer2_voxel));

          // Get the RGB color for this voxel using the provided functor

          // FIXME: add color
        }
      }
    }
  }
}

void LayerConverter::markerMsgFromColorLayer(
  const TsdfLayer & tsdf_layer, const ColorLayer & color_layer, const bool undo_gamma_correction,
  const float max_tsdf_distance, const float min_tsdf_weight, const float exclusion_height_m,
  const float exclusion_radius_m, const Vector3f & exclusion_center,
  visualization_msgs::msg::Marker * marker_msg, const CudaStream & cuda_stream)
{
  std::vector<Index3D> block_indices_to_serialize = tsdf_layer.getBlockIndicesIf(
    BlockFilter(
      exclusion_height_m, exclusion_radius_m, tsdf_layer.block_size(), exclusion_center));
  auto serialized_color_layer =
    color_layer_serializer_.serialize(color_layer, block_indices_to_serialize, cuda_stream);
  auto serialized_tsdf_layer =
    tsdf_layer_serializer_.serialize(tsdf_layer, block_indices_to_serialize, cuda_stream);

  markerMsgFromLayer<TsdfLayer, ColorLayer>(
    serialized_tsdf_layer, serialized_color_layer, color_layer.voxel_size(),
    TsdfVoxelFilter(max_tsdf_distance, min_tsdf_weight), ColorVoxelToRgb(undo_gamma_correction),
    tsdf_layer.block_size(), marker_msg);
}

void LayerConverter::markerMsgFromColorLayer(
  const EsdfLayer & esdf_layer, const ColorLayer & color_layer, const bool undo_gamma_correction,
  const float exclusion_height_m, const float exclusion_radius_m,
  const Vector3f & exclusion_center, visualization_msgs::msg::Marker * marker_msg,
  const CudaStream & cuda_stream)
{
  std::vector<Index3D> block_indices_to_serialize = esdf_layer.getBlockIndicesIf(
    BlockFilter(
      exclusion_height_m, exclusion_radius_m, esdf_layer.block_size(), exclusion_center));
  auto serialized_color_layer =
    color_layer_serializer_.serialize(color_layer, block_indices_to_serialize, cuda_stream);
  auto serialized_esdf_layer =
    esdf_layer_serializer_.serialize(esdf_layer, block_indices_to_serialize, cuda_stream);

  markerMsgFromLayer<EsdfLayer, ColorLayer>(
    serialized_esdf_layer, serialized_color_layer, color_layer.voxel_size(), EsdfVoxelFilter(),
    ColorVoxelToRgb(undo_gamma_correction), esdf_layer.block_size(), marker_msg);
}

void LayerConverter::markerMsgFromOccupancyLayer(
  const OccupancyLayer & occupancy_layer,
  const float exclusion_height_m,
  const float exclusion_radius_m,
  const Vector3f & exclusion_center,
  visualization_msgs::msg::Marker * marker_msg,
  const CudaStream & cuda_stream)
{
  std::vector<Index3D> block_indices_to_serialize = occupancy_layer.getBlockIndicesIf(
    BlockFilter(
      exclusion_height_m, exclusion_radius_m, occupancy_layer.block_size(), exclusion_center));

  auto serialized_occupancy_layer = occupancy_layer_serializer_.serialize(
    occupancy_layer, block_indices_to_serialize, cuda_stream);

  markerMsgFromLayer<OccupancyLayer, OccupancyLayer>(
    serialized_occupancy_layer, serialized_occupancy_layer, occupancy_layer.voxel_size(),
    OccupancyVoxelFilter(), occupancyVoxelToRgb, occupancy_layer.block_size(), marker_msg);
}

void LayerConverter::markerMsgFromTsdfLayer(
  const TsdfLayer & tsdf_layer, const float max_tsdf_distance, const float min_tsdf_weight,
  const float exclusion_height_m, const float exclusion_radius_m,
  const Vector3f & exclusion_center, visualization_msgs::msg::Marker * marker_msg,
  const CudaStream & cuda_stream)
{
  std::vector<Index3D> block_indices_to_serialize = tsdf_layer.getBlockIndicesIf(
    BlockFilter(
      exclusion_height_m, exclusion_radius_m, tsdf_layer.block_size(), exclusion_center));
  auto serialized_tsdf_layer =
    tsdf_layer_serializer_.serialize(tsdf_layer, block_indices_to_serialize, cuda_stream);

  markerMsgFromLayer<TsdfLayer, TsdfLayer>(
    serialized_tsdf_layer, serialized_tsdf_layer,
    tsdf_layer.voxel_size(),
    TsdfVoxelFilter(max_tsdf_distance, min_tsdf_weight),
    tsdfVoxelToRgb, tsdf_layer.block_size(), marker_msg);
}

void LayerConverter::markerMsgFromFreespaceLayer(
  const TsdfLayer & tsdf_layer, const FreespaceLayer & freespace_layer,
  const float max_tsdf_distance, const float min_tsdf_weight, const float exclusion_height_m,
  const float exclusion_radius_m, const Vector3f & exclusion_center,
  visualization_msgs::msg::Marker * marker_msg, const CudaStream & cuda_stream)
{
  std::vector<Index3D> block_indices_to_serialize = freespace_layer.getBlockIndicesIf(
    BlockFilter(
      exclusion_height_m, exclusion_radius_m, freespace_layer.block_size(), exclusion_center));
  auto serialized_freespace_layer = freespace_layer_serializer_.serialize(
    freespace_layer, block_indices_to_serialize, cuda_stream);
  auto serialized_tsdf_layer =
    tsdf_layer_serializer_.serialize(tsdf_layer, block_indices_to_serialize, cuda_stream);

  markerMsgFromLayer<TsdfLayer, FreespaceLayer>(
    serialized_tsdf_layer, serialized_freespace_layer, freespace_layer.voxel_size(),
    TsdfVoxelFilter(max_tsdf_distance, min_tsdf_weight), freespaceVoxelToRgb,
    freespace_layer.block_size(), marker_msg);
}

}  // namespace conversions
}  // namespace nvblox
