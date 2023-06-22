// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include <glog/logging.h>
#include <nvblox/core/internal/warmup_cuda.h>

#include <memory>

#include <ros/ros.h>

#include "nvblox_ros/nvblox_human_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();
  ros::init(argc, argv, "nvblox_human_node");
  ros::NodeHandle nh("~");

  // Warmup CUDA so it doesn't affect our timings *as* much for the first
  // CUDA call.
  nvblox::warmupCuda();

  //std::shared_ptr<nvblox::NvbloxNode> node(new nvblox::NvbloxNode());
  nvblox::NvbloxNode NvbloxNode(nh);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  ros::shutdown();
  return 0;
}
