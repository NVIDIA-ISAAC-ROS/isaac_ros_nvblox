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

#include <glog/logging.h>
#include <nvblox/core/internal/warmup_cuda.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "nvblox_ros/fuser_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();
  rclcpp::init(argc, argv);

  // Warmup CUDA so it doesn't affect our timings *as* much for the first
  // CUDA call.
  nvblox::warmupCuda();

  rclcpp::executors::SingleThreadedExecutor exec;
  std::shared_ptr<nvblox::FuserNode> node(new nvblox::FuserNode());
  exec.add_node(node);
  exec.spin_once();

  // Fuse until we run out of frames.
  bool success = true;
  while (success) {
    // Update the fuser node (e.g. integrating a frame).
    success = node->update();
    // Do ROS stuff after fusing a frame.
    exec.spin_some();
  }

  // Keep up ROS until we shutdown.
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
