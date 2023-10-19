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
