// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <gtest/gtest.h>

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

// White-box testing: access internal state for verification
#define private public
#include "multi_realsense_emitter_synchronizer/emitter_synchronizer.hpp"
#include "multi_realsense_emitter_synchronizer/multi_realsense_emitter_synchronizer_node.hpp"
#undef private

namespace nvblox
{
class RclcppFixture : public ::testing::Test {
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite() {rclcpp::shutdown();}
};

std::shared_ptr<realsense2_camera_msgs::msg::Metadata> make_metadata_msg(
  int64_t t_ns,
  int emitter_mode)
{
  realsense2_camera_msgs::msg::Metadata msg;
  msg.header.stamp.sec = static_cast<int32_t>(t_ns / 1000000000LL);
  msg.header.stamp.nanosec = static_cast<uint32_t>(t_ns % 1000000000LL);
  msg.json_data = std::string("{\"frame_emitter_mode\":") + std::to_string(emitter_mode) + "}";
  return std::make_shared<realsense2_camera_msgs::msg::Metadata>(msg);
}

TEST_F(RclcppFixture, EmitterSynchronizer_HappyPath) {
  auto node = std::make_shared<rclcpp::Node>("emitter_sync_test_node");
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(
    node, "/nonexistent_remote");

  EmitterSynchronizer syncer("test_cam", param_client);
  bool is_synced = false;
  bool state = true;
  // Should sync within 500 iterations.
  for (int i = 0; i < 100; ++i) {
    is_synced = syncer.syncEmitterMode(/*reference*/ state, state);
    if (is_synced) {
      break;
    }
    state = !state;
  }
  EXPECT_TRUE(is_synced);
  EXPECT_EQ(syncer.state_, EmitterSynchronizer::State::SYNCHRONIZED);
}

TEST_F(RclcppFixture, EmitterSynchronizer_ResetTransitionsToFailed) {
  auto node = std::make_shared<rclcpp::Node>("emitter_sync_test_node4");
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/nonexistent_remote");
  EmitterSynchronizer syncer("test_cam4", param_client);
  syncer.attempts_ = std::numeric_limits<int>::max() - 1;
  syncer.resetSyncAttempt();
  EXPECT_EQ(syncer.state_, EmitterSynchronizer::State::FAILED);
}

TEST_F(RclcppFixture, EmitterSynchronizer_Failure_NoToggling) {
  auto node = std::make_shared<rclcpp::Node>("emitter_sync_test_node");
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(
    node, "/nonexistent_remote");
  EmitterSynchronizer syncer("test_cam", param_client);
  // Force one attempt.
  syncer.attempts_ = std::numeric_limits<int>::max() - 1;

  bool is_synced = false;
  for (int i = 0; i < 500; ++i) {
    is_synced = syncer.syncEmitterMode(/*reference*/ true, true);
  }
  EXPECT_FALSE(is_synced);
  EXPECT_EQ(syncer.state_, EmitterSynchronizer::State::FAILED);
}

TEST_F(RclcppFixture, EmitterSynchronizer_Failure_InvalidState) {
  auto node = std::make_shared<rclcpp::Node>("emitter_sync_test_node");
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(
    node, "/nonexistent_remote");

  EmitterSynchronizer syncer("test_cam", param_client);
  // Force one attempt.
  syncer.attempts_ = std::numeric_limits<int>::max() - 1;

  bool is_synced = false;
  bool state = true;
  for (int i = 0; i < 500; ++i) {
    is_synced = syncer.syncEmitterMode(/*reference*/ !state, state);
    state = !state;
  }
  EXPECT_FALSE(is_synced);
  EXPECT_EQ(syncer.state_, EmitterSynchronizer::State::FAILED);
}

TEST_F(RclcppFixture, Node_MetadataParsing_Robustness) {
  // Create the node to access parsing helper.
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      rclcpp::Parameter("camera_names", std::vector<std::string>{"cam0", "cam1"}),
      rclcpp::Parameter("max_sync_tolerance_us", 1000)
  });
  MultiRealsenseEmitterSynchronizerNode node{opts};

  auto make_msg = [](const std::string & json_text) {
      realsense2_camera_msgs::msg::Metadata msg;
      msg.json_data = json_text;
      return std::make_shared<realsense2_camera_msgs::msg::Metadata>(msg);
    };

  // Valid: emitter off (0)
  {
    auto m = make_msg("{\"frame_emitter_mode\":0}");
    auto st = node.getEmitterStateFromMetadataMsg(m);
    EXPECT_EQ(static_cast<int>(st), 0);
  }
  // Valid: emitter on (1)
  {
    auto m = make_msg("{\"frame_emitter_mode\":1}");
    auto st = node.getEmitterStateFromMetadataMsg(m);
    EXPECT_EQ(static_cast<int>(st), 1);
  }
  // Invalid value -> kUnknown
  {
    auto m = make_msg("{\"frame_emitter_mode\":123}");
    auto st = node.getEmitterStateFromMetadataMsg(m);
    EXPECT_EQ(static_cast<int>(st), 2);
  }
  // Missing key -> kUnknown
  {
    auto m = make_msg("{\"other_key\":1}");
    auto st = node.getEmitterStateFromMetadataMsg(m);
    EXPECT_EQ(static_cast<int>(st), 2);
  }
  // Malformed JSON -> kUnknown
  {
    auto m = make_msg("{\"frame_emitter_mode\":");
    auto st = node.getEmitterStateFromMetadataMsg(m);
    EXPECT_EQ(static_cast<int>(st), 2);
  }
}


TEST_F(RclcppFixture, Node_SyncCallback_SetsSynchronizedAfterInSyncFrames) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      rclcpp::Parameter("camera_names", std::vector<std::string>{"cam0", "cam1"}),
      rclcpp::Parameter("max_sync_tolerance_us", 1000)
  });

  MultiRealsenseEmitterSynchronizerNode node{opts};

  const int64_t frame_period_ns = 16666666;  // ~60 FPS
  // Provide some rames to increment count.
  for (int i = 0; i < 10; ++i) {
    int64_t t = i * frame_period_ns;
    node.syncCallback({make_metadata_msg(t, 1), make_metadata_msg(t, 1)});
  }
  EXPECT_GT(node.sync_count_, 0);
  EXPECT_FALSE(node.is_messages_synchronized_);

  for (int i = 10; i < MultiRealsenseEmitterSynchronizerNode::kConsecutiveSyncsRequired; ++i) {
    int64_t t = i * frame_period_ns;
    auto m0 = make_metadata_msg(t, 1);
    auto m1 = make_metadata_msg(t, 1);
    node.syncCallback({m0, m1});
  }

  EXPECT_TRUE(node.is_messages_synchronized_);
}

TEST_F(RclcppFixture, Node_SyncCallback_ResetsOnOutOfToleranceFrame) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      rclcpp::Parameter("camera_names", std::vector<std::string>{"cam0", "cam1"}),
      rclcpp::Parameter("max_sync_tolerance_us", 1000)  // 1 ms
  });

  MultiRealsenseEmitterSynchronizerNode node{opts};
  const int64_t frame_period_ns = 16666666;

  // Provide some in-sync frames to increment count.
  for (int i = 0; i < 10; ++i) {
    int64_t t = i * frame_period_ns;
    node.syncCallback({make_metadata_msg(t, 1), make_metadata_msg(t, 1)});
  }
  EXPECT_GT(node.sync_count_, 0);

  // Now provide one out-of-tolerance pair: 2 ms apart (> 1 ms threshold)
  {
    int64_t t = 10 * frame_period_ns;
    auto m0 = make_metadata_msg(t, 1);
    auto m1 = make_metadata_msg(t + 2000000LL, 1);
    node.syncCallback({m0, m1});
  }

  // After out of sync message, reset sync counter.
  EXPECT_EQ(node.sync_count_, 0);
  EXPECT_FALSE(node.is_messages_synchronized_);
}

}  // namespace nvblox
