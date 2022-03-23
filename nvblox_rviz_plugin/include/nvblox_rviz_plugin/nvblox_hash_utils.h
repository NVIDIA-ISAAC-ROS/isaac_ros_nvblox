/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
 
#pragma once

#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <nvblox_msgs/msg/index3_d.hpp>

namespace nvblox_rviz_plugin {

// Minimal functions taken from nvblox/core/hash.h to remove dependency and
// uses the ROS types instead.

typedef nvblox_msgs::msg::Index3D Index3D;

/**
 * Performs deco hashing on block indexes. Based on recommendations of
 * "Investigating the impact of Suboptimal Hashing Functions" by L. Buckley et
 * al.
 */
struct Index3DHash {
  /// number was arbitrarily chosen with no good justification
  static constexpr size_t sl = 17191;
  static constexpr size_t sl2 = sl * sl;

  std::size_t operator()(const Index3D& index) const {
    return static_cast<unsigned int>(index.x + index.y * sl + index.z * sl2);
  }
};

template <typename ValueType>
struct Index3DHashMapType {
  typedef std::unordered_map<Index3D, ValueType, Index3DHash,
                             std::equal_to<Index3D>>
      type;
};

}  // namespace nvblox_rviz_plugin
