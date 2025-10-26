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

#include "nvblox_ros/layer_publishing.hpp"
#include <rclcpp/rclcpp.hpp>
#include "nvblox_ros/conversions/mesh_conversions.hpp"

namespace nvblox
{

bool hasSubscriber(
  const typename rclcpp::Publisher<nvblox_msgs::msg::VoxelBlockLayer>::SharedPtr pub)
{
  return pub && pub->get_subscription_count() > 0;
}

bool hasSubscriber(
  const typename rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub)
{
  return pub && pub->get_subscription_count() > 0;
}

template<typename T>
T clamp(const T value, const T minval, const T maxval)
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
// An optional undoing of gamma correction can be performed to prevent
// washed-out colors for rendering pipelines that already does gamma correction
// (e.g. Foxglove).
class ColorVoxelToRgb
{
public:
  std_msgs::msg::ColorRGBA operator()(const ColorVoxel & voxel)
  {
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    if (undo_gamma_correction_) {
      color.r = undo_gamma_correction_lut_[voxel.color.r()];
      color.g = undo_gamma_correction_lut_[voxel.color.g()];
      color.b = undo_gamma_correction_lut_[voxel.color.b()];
    } else {
      color.r = static_cast<float>(voxel.color.r()) / 255.F;
      color.g = static_cast<float>(voxel.color.g()) / 255.F;
      color.b = static_cast<float>(voxel.color.b()) / 255.F;
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

      // Constants according to the sRGB standard: https://en.wikipedia.org/wiki/SRGB
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
  std_msgs::msg::ColorRGBA color;
  constexpr float kMaxDurationMs = 5000.F;
  const int64_t duration =
    static_cast<int64_t>(voxel.consecutive_occupancy_duration_ms);

  return rgbFromScalar(
    std::max(1.F - static_cast<float>(duration) / kMaxDurationMs, 0.0F));
}

// Functor that returns true for occupancy voxels with an log_odds above a given
// threshold.
class OccupancyVoxelFilter
{
public:
  explicit OccupancyVoxelFilter(const float min_log_odds = 1E-3F)
  : min_log_odds_(min_log_odds) {}

  bool operator()(const OccupancyVoxel & voxel)
  {
    return voxel.log_odds > min_log_odds_;
  }

private:
  float min_log_odds_;
};

// Functor that returns true for freespance voxels with high confidence.
class FreespaceVoxelFilter
{
public:
  FreespaceVoxelFilter() = default;
  bool operator()(const FreespaceVoxel & voxel)
  {
    return !voxel.is_high_confidence_freespace;
  }
};

// Functor that returns true for tsdf voxels within specified distance and
// weights thresholds.
class TsdfVoxelFilter
{
public:
  explicit TsdfVoxelFilter(const float min_tsdf_weight)
  : min_tsdf_weight_(min_tsdf_weight) {}

  bool operator()(const TsdfVoxel & voxel)
  {
    // NOTE: this corresponds to is_inside && is_observed in the esdf case.
    return voxel.distance <= 0.f && voxel.weight > min_tsdf_weight_;
  }

private:
  float min_tsdf_weight_;
};

// Functor that returns true for esdf voxels on the surface
class EsdfVoxelFilter
{
public:
  EsdfVoxelFilter() = default;

  bool operator()(const EsdfVoxel & voxel) {return voxel.is_inside && voxel.observed;}
};

// Functor that returns true for blocks within an exclusion radius and below a
// given exclusion height.
class BlockFilter
{
public:
  BlockFilter(
    const float exclusion_height_m, const float exclusion_radius_m,
    const float block_size, const Vector3f & exclusion_center)
  : exclusion_height_m_(exclusion_height_m),
    exclusion_radius_sqm_(exclusion_radius_m * exclusion_radius_m),
    block_size_(block_size),
    exclusion_center_(exclusion_center) {}

  bool operator()(const Index3D & block_index)
  {
    const Vector3f block_position =
      getCenterPositionFromBlockIndex(block_size_, block_index);
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


void createBlockRemovalMsg(
  const std::vector<Index3D> & blocks_to_remove, const std::string & frame_id,
  const rclcpp::Time & timestamp, const float block_size, const float voxel_size,
  nvblox_msgs::msg::VoxelBlockLayer * msg)
{
  const size_t num_blocks = blocks_to_remove.size();

  msg->header.stamp = timestamp;
  msg->header.frame_id = frame_id;
  msg->block_size_m = block_size;
  msg->voxel_size_m = voxel_size;
  msg->block_indices.resize(num_blocks);
  msg->blocks.resize(num_blocks);

  // Add empty blocks for each index that should be deleted
  for (size_t i_block = 0; i_block < num_blocks; ++i_block) {
    const Index3D & block_index = blocks_to_remove[i_block];
    msg->block_indices[i_block] = conversions::index3DMessageFromIndex3D(block_index);
    msg->blocks[i_block] = nvblox_msgs::msg::VoxelBlock();
  }
}

// Convert layer to cubelist marker message for visualization
//
// The function takes two layers as input:
//   * Layer1 is used to filter out voxels we don't want to visualize
//   * Layer2 is used to determine the RGB for coloring the voxels
//
// This is useful if we e.g. want to visualize a layer without any notion of
// geometry (such as the color layer). Note that it is possible to use the same input
// for both layer1 and layer2.
//
// @param serialized_layer1         Layer used for filtering unwanted voxels
// @param serialized_layer2         Layer used for visualizing color
// @param voxel_size                Size of visualized voxels
// @param voxel_in_layer1_valid     Functor that returns true if a voxel in
// layer1 should be displayed
// @param voxel_in_layer2_to_color  Functor that converts a voxel in layer2 to a
// color
// @param block_size                Block size of visualized layer
// @param marker_msg                Resulting pointcloud message
template<typename LayerType1, typename LayerType2>
void publishVoxelLayerUsingMarker(
  const std::shared_ptr<const SerializedLayer<typename LayerType1::VoxelType>> & serialized_layer1,
  const std::shared_ptr<const SerializedLayer<typename LayerType2::VoxelType>> & serialized_layer2,
  const std::string frame_id, const rclcpp::Time timestamp,
  const float block_size, const float voxel_size,
  const std::function<bool(typename LayerType1::VoxelType)> voxel_in_layer1_valid,
  const std::function<std_msgs::msg::ColorRGBA(
    typename LayerType2::VoxelType)>
  voxel_in_layer2_to_color,
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
  publisher)
{
  if (!hasSubscriber(publisher)) {
    return;
  }

  visualization_msgs::msg::Marker marker_msg;
  const std::vector<Index3D> & block_indices = serialized_layer1->block_indices;

  constexpr int kVoxelsPerSide = LayerType1::VoxelBlockType::kVoxelsPerSide;
  static_assert(
    kVoxelsPerSide == LayerType2::VoxelBlockType::kVoxelsPerSide,
    "Layers must have same block dimension");

  const int max_num_points =
    block_indices.size() * kVoxelsPerSide * kVoxelsPerSide * kVoxelsPerSide;

  // All voxel are placed in the same marker message. This means that all voxels will be updated
  // every time we publish
  marker_msg.header.frame_id = frame_id;
  marker_msg.header.stamp = timestamp;
  marker_msg.points.reserve(max_num_points);
  marker_msg.ns = "layer";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.lifetime = rclcpp::Duration(0, 0);  // never decay. only update/add
  marker_msg.frame_locked = false;

  // Subtract a small amount from the scale to prevent clipping when rendering
  const float scale = voxel_size - 1E-3;
  marker_msg.scale.x = scale;
  marker_msg.scale.y = scale;
  marker_msg.scale.z = scale;

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

          // Get position of this voxel from the indices
          const Vector3f pos =
            getCenterPositionFromBlockIndexAndVoxelIndex(block_size, block_index, {x, y, z});

          geometry_msgs::msg::Point point_msg;
          point_msg.x = pos(0);
          point_msg.y = pos(1);
          point_msg.z = pos(2);
          marker_msg.points.emplace_back(point_msg);

          // Get the RGB color for this voxel using the provided functor
          marker_msg.colors.emplace_back(voxel_in_layer2_to_color(layer2_voxel));
        }
      }
    }
  }
  publisher->publish(marker_msg);
}


template<typename LayerType>
std::tuple<int, int> getOffsetAndNumVoxelsForBlock(
  const std::optional<std::shared_ptr<const SerializedLayer<
    typename LayerType::VoxelType>>> & serialized_layer, const int block_index)
{
  if (!serialized_layer) {
    return std::make_tuple(-1, -1);
  }

  CHECK(block_index < static_cast<int>(serialized_layer.value()->block_offsets.size()) - 1);
  const int offset = serialized_layer.value()->block_offsets[block_index];
  const int num_voxels = serialized_layer.value()->block_offsets[block_index + 1] - offset;
  return std::make_tuple(offset, num_voxels);
}


// Convert layer to cubelist marker message for visualization
//
// The function accepts two layers.
//   * Layer1 is used to filter out voxels we don't want to visualize
//   * Layer2 is used to determine the RGB for coloring the voxels
//
// This is useful if we e.g. want to visualize a layer without any notion of
// geometry (e.g. the color layer). Note that is possible give the same layer
// for both layer1 and layer2.
//
// @param serialized_layer1         Layer used for filtering unwanted voxels
// @param serialized_layer2         Layer used for visualizing color
// @param freespace_layer           Optional freespace layer for filtering occupied voxels
// @param voxel_size                Size of visualized voxels
// @param voxel_in_layer1_valid     Functor that returns true if a voxel in
// layer1 should be displayed
// @param voxel_in_layer2_to_color  Functor that converts a voxel in layer2 to a
// color
// @param block_size                Block size of visualized layer
// @param marker_msg                Resulting pointcloud message
template<typename LayerType1, typename LayerType2>
void publishVoxelLayerUsingPlugin(
  const std::shared_ptr<const SerializedLayer<
    typename LayerType1::VoxelType>> & serialized_layer1,
  const std::shared_ptr<const SerializedLayer<
    typename LayerType2::VoxelType>> & serialized_layer2,
  const std::optional<std::shared_ptr<const SerializedFreespaceLayer>>
  freespace_layer,
  const std::vector<Index3D> & blocks_to_remove, const float block_size,
  const float voxel_size, const std::string & frame_id, const LayerType layer_type,
  const rclcpp::Time & timestamp,
  const std::function<bool(typename LayerType1::VoxelType)>
  voxel_in_layer1_valid,
  const std::function<
    std_msgs::msg::ColorRGBA(typename LayerType2::VoxelType)>
  voxel_in_layer2_to_color,
  const rclcpp::Publisher<nvblox_msgs::msg::VoxelBlockLayer>::SharedPtr
  publisher)
{
  if (!hasSubscriber(publisher)) {
    return;
  }

  /// Remove blocks
  nvblox_msgs::msg::VoxelBlockLayer delete_msg;
  createBlockRemovalMsg(blocks_to_remove, frame_id, timestamp, block_size, voxel_size, &delete_msg);
  publisher->publish(delete_msg);

  /// Add/update blocks
  nvblox_msgs::msg::VoxelBlockLayer update_msg;

  const std::vector<Index3D> & block_indices = serialized_layer1->block_indices;
  const int num_blocks = block_indices.size();

  update_msg.clear = false;  // TODO(tingdahl) make this an option
  update_msg.header.stamp = timestamp;
  update_msg.header.frame_id = frame_id;
  update_msg.block_size_m = block_size;
  update_msg.voxel_size_m = voxel_size;
  update_msg.block_indices.resize(num_blocks);
  update_msg.blocks.resize(num_blocks);
  update_msg.layer_type = static_cast<int>(layer_type);

  constexpr int kVoxelsPerSide = LayerType1::VoxelBlockType::kVoxelsPerSide;
  constexpr int kNumVoxels = LayerType1::VoxelBlockType::kNumVoxels;
  static_assert(
    kVoxelsPerSide == LayerType2::VoxelBlockType::kVoxelsPerSide,
    "Layers must have same block dimension");

  // Need same number of blocks in both layers. This should be guaranteed if the
  // layers where serialized at the same time.
  CHECK_EQ(
    serialized_layer1->block_indices.size(),
    serialized_layer2->block_indices.size());
  if (freespace_layer) {
    CHECK_EQ(
      freespace_layer.value()->block_indices.size(),
      serialized_layer1->block_indices.size());
  }

  FreespaceVoxelFilter freespace_voxel_filter;

  // Go over all blocks
  timing::Timer block_timer("ros/publish_layer/block_loop");
  for (size_t i_block = 0; i_block < block_indices.size(); ++i_block) {
    const Index3D & block_index = block_indices[i_block];
    update_msg.block_indices[i_block] = conversions::index3DMessageFromIndex3D(block_index);
    update_msg.blocks[i_block] = nvblox_msgs::msg::VoxelBlock();

    //  For the current block: get the offset in the serialized vectors and the number of serialized
    //  voxels in the block.
    auto [offset_layer1, num_layer1] = getOffsetAndNumVoxelsForBlock<LayerType1>(
      serialized_layer1,
      i_block);

    auto [offset_layer2, num_layer2] = getOffsetAndNumVoxelsForBlock<LayerType2>(
      serialized_layer2,
      i_block);

    auto [offset_freespace_layer, num_freespace_layer] =
      getOffsetAndNumVoxelsForBlock<FreespaceLayer>(freespace_layer, i_block);

    // The number of voxels in a block is guaranteed to be zero or kNumVoxels
    // (we only allocate full blocks).
    // If the block of layer 1 is not available we skip publishing the voxels.
    // In contrast, we still publish voxels with missing block in layer 2.
    // Layer 2 is used to color the voxel visualization and
    // if it is missing we use the default color.
    if (num_layer1 != kNumVoxels) {
      continue;
    }
    if (freespace_layer && num_freespace_layer != num_layer1) {
      continue;
    }

    // For each voxel in the block...
    for (int x = 0; x < kVoxelsPerSide; ++x) {
      for (int y = 0; y < kVoxelsPerSide; ++y) {
        for (int z = 0; z < kVoxelsPerSide; ++z) {
          const int lin_index = z + 8 * y + 64 * x;
          const typename LayerType1::VoxelType & layer1_voxel =
            serialized_layer1->voxels[offset_layer1 + lin_index];
          // Reject voxels using the provided functor
          if (!voxel_in_layer1_valid(layer1_voxel)) {
            continue;
          }

          if (freespace_layer) {
            const FreespaceVoxel & freespace_voxel =
              freespace_layer.value()->voxels[offset_freespace_layer + lin_index];
            // Don't visualize voxels free voxels according to the Freespace layer.
            // Note that that freespace voxel filter returns "false" for free voxels.
            if (!freespace_voxel_filter(freespace_voxel)) {
              continue;
            }
          }

          typename LayerType2::VoxelType layer2_voxel;
          if (num_layer2 == kNumVoxels) {
            // Voxels are available.
            layer2_voxel = serialized_layer2->voxels[offset_layer2 + lin_index];
          }

          // Get the voxel centers from the indices
          geometry_msgs::msg::Point32 point_msg;
          point_msg.x = block_index.x() * block_size + x * voxel_size + voxel_size / 2.f;
          point_msg.y = block_index.y() * block_size + y * voxel_size + voxel_size / 2.f;
          point_msg.z = block_index.z() * block_size + z * voxel_size + voxel_size / 2.f;
          update_msg.blocks[i_block].centers.emplace_back(point_msg);

          update_msg.blocks[i_block].colors.emplace_back(
            voxel_in_layer2_to_color(layer2_voxel));
        }
      }
    }
  }
  block_timer.Stop();

  timing::Timer publish_timer("ros/publish_layer/publish");
  publisher->publish(update_msg);
  publish_timer.Stop();
}

void LayerPublisher::publishMesh(
  std::shared_ptr<SerializedColorMeshLayer> serialized_mesh,
  const std::vector<Index3D> & blocks_to_remove, const float block_size,
  const std::string & frame_id, const rclcpp::Time & timestamp,
  const rclcpp::Logger & logger)
{
  bool serialize_full_mesh = false;
  size_t new_subscriber_count = mesh_publisher_->get_subscription_count();

  // In case we have new subscribers, publish the ENTIRE map once.
  nvblox_msgs::msg::Mesh mesh_msg;
  if (new_subscriber_count > mesh_subscriber_count_) {
    RCLCPP_INFO(logger, "Got a new subscriber, sending entire map.");
    serialize_full_mesh = true;
  }
  mesh_subscriber_count_ = new_subscriber_count;

  RCLCPP_DEBUG_STREAM(
    logger, "Num of streamed mesh blocks: "
      << serialized_mesh->block_indices.size());

  // Publish the mesh updates.
  if (mesh_subscriber_count_ > 0) {
    if (!blocks_to_remove.empty()) {
      nvblox_msgs::msg::Mesh deleted_mesh_msg;
      conversions::meshMessageFromBlocksToDelete(
        blocks_to_remove, timestamp, frame_id, block_size, &deleted_mesh_msg);
      mesh_publisher_->publish(deleted_mesh_msg);
    }

    // Publish mesh blocks from serialized mesh
    if (!serialized_mesh->block_indices.empty()) {
      const bool resend_full_mesh = serialize_full_mesh;
      conversions::meshMessageFromSerializedMesh(
        serialized_mesh, timestamp,
        frame_id, block_size,
        resend_full_mesh, &mesh_msg);
      mesh_publisher_->publish(mesh_msg);
    }
  }
}
LayerPublisher::LayerPublisher(
  const MappingType mapping_type,
  const float min_tsdf_weight,
  const float exclusion_height_m,
  const float exclusion_radius_m,
  rclcpp::Node * node)
: min_tsdf_weight_(min_tsdf_weight),
  exclusion_height_m_(exclusion_height_m),
  exclusion_radius_m_(exclusion_radius_m)
{
  // Mesh publishers
  mesh_publisher_ = node->create_publisher<nvblox_msgs::msg::Mesh>("~/mesh", 1);

  // Layer publishers
  if (!isStaticOccupancy(mapping_type)) {
    tsdf_layer_publisher_plugin_ =
      node->create_publisher<nvblox_msgs::msg::VoxelBlockLayer>(
      "~/tsdf_layer", 1);

    tsdf_layer_publisher_marker_ =
      node->create_publisher<visualization_msgs::msg::Marker>(
      "~/tsdf_layer_marker", 1);
  }
  color_layer_publisher_plugin_ =
    node->create_publisher<nvblox_msgs::msg::VoxelBlockLayer>(
    "~/color_layer", 1);
  color_layer_publisher_marker_ =
    node->create_publisher<visualization_msgs::msg::Marker>(
    "~/color_layer_marker", 1);


  if (isDynamicMapping(mapping_type)) {
    freespace_layer_publisher_plugin_ =
      node->create_publisher<nvblox_msgs::msg::VoxelBlockLayer>(
      "~/freespace_layer", 1);
    freespace_layer_publisher_marker_ =
      node->create_publisher<visualization_msgs::msg::Marker>(
      "~/freespace_layer_marker", 1);
  }

  if (isDynamicMapping(mapping_type) || isHumanMapping(mapping_type)) {
    // Dynamic occupancy
    dynamic_occupancy_layer_publisher_plugin_ =
      node->create_publisher<nvblox_msgs::msg::VoxelBlockLayer>(
      "~/dynamic_occupancy_layer", 1);
    dynamic_occupancy_layer_publisher_marker_ =
      node->create_publisher<visualization_msgs::msg::Marker>(
      "~/dynamic_occupancy_layer_marker", 1);
  }
}


LayerTypeBitMask LayerPublisher::getLayersToStreamBitMask()
{
  LayerTypeBitMask mask;

  if (hasSubscriber(color_layer_publisher_plugin_) ||
    hasSubscriber(color_layer_publisher_marker_))
  {
    mask |= LayerType::kColor;
  }

  if (hasSubscriber(tsdf_layer_publisher_plugin_) ||
    hasSubscriber(tsdf_layer_publisher_marker_))
  {
    mask |= LayerType::kTsdf;
  }

  if (mesh_publisher_ && mesh_publisher_->get_subscription_count() > 0) {
    mask |= LayerType::kColorMesh;
  } else {
    mesh_subscriber_count_ = 0;
  }

  if (hasSubscriber(dynamic_occupancy_layer_publisher_plugin_) ||
    hasSubscriber(dynamic_occupancy_layer_publisher_marker_))
  {
    mask |= LayerType::kOccupancy;
  }

  if (hasSubscriber(freespace_layer_publisher_plugin_) ||
    hasSubscriber(freespace_layer_publisher_marker_))
  {
    mask |= LayerType::kFreespace;
  }

  return mask;
}

void LayerPublisher::serializeAndpublishSubscribedLayers(
  const Transform & T_L_C, const std::string & frame_id, rclcpp::Time timestamp,
  const float layer_streamer_bandwidth_limit_mbps,
  std::shared_ptr<Mapper> static_mapper,
  std::shared_ptr<Mapper> dynamic_mapper, const rclcpp::Logger & logger)
{
  timing::Timer publish_layer_timer("ros/publish_layer");

  CHECK_NOTNULL(static_mapper);
  LayerTypeBitMask layers_to_stream = getLayersToStreamBitMask();

  /// Mesh is only computed when we're serializing
  if (layers_to_stream & LayerType::kColorMesh) {
    static_mapper->updateColorMesh();
  }

  timing::Timer serialize_timer("ros/publish_layer/serialize");

  // If we're streaming color, we also need to serialize the TSDF layer to get the geometry and the
  // Freespace layer to exclude occupied voxels. Note that the Freespace layer is optional.
  LayerTypeBitMask layers_to_serialize = layers_to_stream;
  if (layers_to_serialize & LayerType::kColor) {
    layers_to_serialize |= LayerType::kTsdf;
    layers_to_serialize |= LayerType::kFreespace;
  }


  BlockExclusionParams block_exclusion_params{
    .exclusion_center_m = T_L_C.translation(),
    .exclusion_height_m = exclusion_height_m_,
    .exclusion_radius_m = exclusion_radius_m_,
    .block_size_m = static_mapper->tsdf_layer().block_size(),
  };

  static_mapper->serializeSelectedLayers(
    layers_to_serialize, layer_streamer_bandwidth_limit_mbps,
    block_exclusion_params);

  serialize_timer.Stop();

  const std::vector<Index3D> blocks_to_remove_static_mapper =
    static_mapper->getClearedBlocks({});

  std::optional<std::shared_ptr<const SerializedFreespaceLayer>> kNoFreespaceLayerExclusion =
    std::nullopt;

  if (layers_to_stream & LayerType::kTsdf) {
    timing::Timer publish_timer("ros/publish_tsdf_layer");
    publishVoxelLayerUsingPlugin<TsdfLayer, TsdfLayer>(
      static_mapper->serializedTsdfLayer(),
      static_mapper->serializedTsdfLayer(), kNoFreespaceLayerExclusion,
      blocks_to_remove_static_mapper,
      static_mapper->tsdf_layer().block_size(),
      static_mapper->tsdf_layer().voxel_size(), frame_id, LayerType::kTsdf, timestamp,
      TsdfVoxelFilter(min_tsdf_weight_), tsdfVoxelToRgb,
      tsdf_layer_publisher_plugin_);

    publishVoxelLayerUsingMarker<TsdfLayer, TsdfLayer>(
      static_mapper->serializedTsdfLayer(),
      static_mapper->serializedTsdfLayer(), frame_id, timestamp,
      static_mapper->tsdf_layer().block_size(),
      static_mapper->tsdf_layer().voxel_size(),
      TsdfVoxelFilter(min_tsdf_weight_), tsdfVoxelToRgb,
      tsdf_layer_publisher_marker_);
  }

  if (layers_to_stream & LayerType::kColor) {
    timing::Timer publish_timer("ros/publish_color_layer");

    std::optional<std::shared_ptr<const SerializedFreespaceLayer>> optional_freespace_layer =
      kNoFreespaceLayerExclusion;

    if (static_mapper->projective_layer_type() == ProjectiveLayerType::kTsdfWithFreespace) {
      optional_freespace_layer = static_mapper->serializedFreespaceLayer();
    }

    publishVoxelLayerUsingPlugin<TsdfLayer, ColorLayer>(
      static_mapper->serializedTsdfLayer(),
      static_mapper->serializedColorLayer(),
      optional_freespace_layer,
      blocks_to_remove_static_mapper,
      static_mapper->tsdf_layer().block_size(),
      static_mapper->tsdf_layer().voxel_size(), frame_id, LayerType::kColor, timestamp,
      TsdfVoxelFilter(min_tsdf_weight_), ColorVoxelToRgb(),
      color_layer_publisher_plugin_);

    publishVoxelLayerUsingMarker<TsdfLayer, ColorLayer>(
      static_mapper->serializedTsdfLayer(),
      static_mapper->serializedColorLayer(), frame_id, timestamp,
      static_mapper->tsdf_layer().block_size(),
      static_mapper->tsdf_layer().voxel_size(),
      TsdfVoxelFilter(min_tsdf_weight_), ColorVoxelToRgb(),
      color_layer_publisher_marker_);
  }

  if (layers_to_stream & LayerType::kColorMesh) {
    timing::Timer publish_timer("ros/publish_mesh_layer");
    publishMesh(
      static_mapper->serializedColorMeshLayer(), blocks_to_remove_static_mapper,
      static_mapper->color_mesh_layer().block_size(), frame_id, timestamp,
      logger);
  }

  if (layers_to_stream & LayerType::kFreespace) {
    timing::Timer publish_timer("ros/publish_freespace_layer");
    publishVoxelLayerUsingPlugin<FreespaceLayer, FreespaceLayer>(
      static_mapper->serializedFreespaceLayer(),
      static_mapper->serializedFreespaceLayer(), kNoFreespaceLayerExclusion,
      blocks_to_remove_static_mapper,
      static_mapper->freespace_layer().block_size(),
      static_mapper->freespace_layer().voxel_size(), frame_id, LayerType::kFreespace, timestamp,
      FreespaceVoxelFilter(), freespaceVoxelToRgb,
      freespace_layer_publisher_plugin_);

    publishVoxelLayerUsingMarker<FreespaceLayer, FreespaceLayer>(
      static_mapper->serializedFreespaceLayer(),
      static_mapper->serializedFreespaceLayer(), frame_id, timestamp,
      static_mapper->freespace_layer().block_size(),
      static_mapper->freespace_layer().voxel_size(),
      FreespaceVoxelFilter(), freespaceVoxelToRgb,
      freespace_layer_publisher_marker_);
  }

  if (dynamic_mapper != nullptr && (layers_to_stream & LayerType::kOccupancy)) {
    dynamic_mapper->serializeSelectedLayers(
      LayerType::kOccupancy, layer_streamer_bandwidth_limit_mbps,
      block_exclusion_params);

    const std::vector<Index3D> blocks_to_remove_dynamic_mapper =
      dynamic_mapper->getClearedBlocks({});

    timing::Timer publish_timer("ros/publish_occupancy_layer");
    publishVoxelLayerUsingPlugin<OccupancyLayer, OccupancyLayer>(
      dynamic_mapper->serializedOccupancyLayer(),
      dynamic_mapper->serializedOccupancyLayer(), kNoFreespaceLayerExclusion,
      blocks_to_remove_dynamic_mapper,
      dynamic_mapper->occupancy_layer().block_size(),
      dynamic_mapper->occupancy_layer().voxel_size(), frame_id, LayerType::kOccupancy, timestamp,
      OccupancyVoxelFilter(), occupancyVoxelToRgb,
      dynamic_occupancy_layer_publisher_plugin_);

    publishVoxelLayerUsingMarker<OccupancyLayer, OccupancyLayer>(
      dynamic_mapper->serializedOccupancyLayer(),
      dynamic_mapper->serializedOccupancyLayer(), frame_id, timestamp,
      dynamic_mapper->occupancy_layer().block_size(),
      dynamic_mapper->occupancy_layer().voxel_size(),
      OccupancyVoxelFilter(), occupancyVoxelToRgb,
      dynamic_occupancy_layer_publisher_marker_);
  }

  publish_layer_timer.Stop();
}

}  // namespace nvblox
