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
#pragma once

#include <string>

namespace nvblox
{

// NOTE(alexmillane): This file is almost a duplicate of the dataset_type.h file in nvblox_core.
// Because we add a ROS specific dataset type, we have to duplicate some of the information here.

enum class RosDatasetType { kThreedMatch, kRedwood, kReplica, kRosbag };

inline std::string toString(const RosDatasetType & dataset_type)
{
  switch (dataset_type) {
    case RosDatasetType::kThreedMatch:
      return "kThreedMatch";
      break;
    case RosDatasetType::kRedwood:
      return "kRedwood";
      break;
    case RosDatasetType::kReplica:
      return "kReplica";
      break;
    case RosDatasetType::kRosbag:
      return "kRosbag";
      break;
    default:
      CHECK(false) << "Requested dataset type is not implemented.";
  }
}

}  // namespace nvblox
