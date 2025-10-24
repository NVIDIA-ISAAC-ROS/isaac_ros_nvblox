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

#include <filesystem>
#include <optional>

#include "glog/logging.h"
#include "gtest/gtest.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "nvblox_ros/rosbag_data_loader.hpp"

namespace nvblox
{

TEST(RosbagDataLoaderTest, DataLoader) {
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("nvblox_ros");
  #ifdef __USE_BAZEL_TEST_CASE_PATHING
  std::string rosbag_path = package_share_directory +
    "/test/integration_tests/test_cases/rosbags/nvblox_pol";
  #else
  std::string rosbag_path = package_share_directory + "/test_cases/rosbags/nvblox_pol";
  #endif
  LOG(INFO) << "nvblox_ros package_share_directory: " << package_share_directory;
  LOG(INFO) << "test rosbag_path: " << rosbag_path;

  EXPECT_TRUE(
    std::filesystem::exists(package_share_directory) &&
    std::filesystem::is_directory(package_share_directory));
  EXPECT_TRUE(std::filesystem::exists(rosbag_path) && std::filesystem::is_directory(rosbag_path));

  const std::string depth_topic = "/front_stereo_camera/depth/ground_truth";
  const std::string depth_camera_info_topic = "/front_stereo_camera/depth/camera_info";
  const std::string color_topic = "/front_stereo_camera/left/image_raw";
  const std::string color_camera_info_topic = "/front_stereo_camera/left/camera_info";
  const std::string world_frame_id = "odom";
  // For the very short test dataset we set the preload time to zero.
  constexpr float kTfPreloadTimeS = 0.1;
  datasets::ros::RosDataLoader data_loader(rosbag_path, depth_topic, depth_camera_info_topic,
    color_topic, color_camera_info_topic, world_frame_id,
    kTfPreloadTimeS);

  DepthImage depth_frame(MemoryType::kDevice);
  ColorImage color_frame(MemoryType::kDevice);
  Transform T_L_C;
  Camera camera;

  // Load the whole bag
  datasets::DataLoadResult load_result = datasets::DataLoadResult::kSuccess;
  int num_loaded = 0;
  while (load_result == datasets::DataLoadResult::kSuccess) {
    load_result = data_loader.loadNext(&depth_frame, &T_L_C, &camera, &color_frame);
    ++num_loaded;
  }
  // NOTE(alexmillane): Right now the test_bag contains 11 messages all with
  // exact matches.
  // Because we pre-load the tf by 0.1 seconds, we don't get all of them. I
  // tested and empirically we get 8 at the moment.
  // If the test ROSbag changes, this number will need to be updated.
  constexpr int kNumMatchingMessagesExpected = 8;
  LOG(INFO) << "Num messages expected: " << kNumMatchingMessagesExpected;
  LOG(INFO) << "Num messages found: " << num_loaded;
  EXPECT_GE(num_loaded, kNumMatchingMessagesExpected);
}

int main(int argc, char ** argv)
{
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace nvblox
