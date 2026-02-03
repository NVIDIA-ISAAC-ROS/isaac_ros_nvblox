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
//
// SPDX-License-Identifier: Apache-2.0

#include "multi_realsense_emitter_synchronizer/multi_realsense_emitter_synchronizer_node.hpp"

#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>
#include <isaac_ros_common/qos.hpp>

#include "multi_realsense_emitter_synchronizer/emitter_synchronizer.hpp"

using json = nlohmann::json;

namespace nvblox
{

MultiRealsenseEmitterSynchronizerNode::EmitterState MultiRealsenseEmitterSynchronizerNode::
getEmitterStateFromMetadataMsg(
  const realsense2_camera_msgs::msg::Metadata::ConstSharedPtr & metadata) const
{
  try {
    const json & json_data = json::parse(metadata->json_data);
    if (json_data.contains("frame_emitter_mode")) {
      int emitter_mode = json_data["frame_emitter_mode"].get<int>();
      if (emitter_mode == 0 || emitter_mode == 1) {
        return static_cast<EmitterState>(emitter_mode);
      } else {
        RCLCPP_WARN(
          get_logger(), "Realsense frame metadata contains invalid emitter mode: %d.",
          emitter_mode);
        return EmitterState::kUnknown;
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Realsense frame metadata did not contain \"frame_emitter_mode\".");
      return EmitterState::kUnknown;
    }
  } catch (const json::parse_error & e) {
    RCLCPP_ERROR(
      get_logger(), "Failed to parse metadata JSON: %s. JSON data: %s", e.what(),
      metadata->json_data.c_str());
    return EmitterState::kUnknown;
  } catch (const json::type_error & e) {
    RCLCPP_ERROR(
      get_logger(),
      "Unexpected type for \"frame_emitter_mode\": %s. JSON data: %s",
      e.what(), metadata->json_data.c_str());
    return EmitterState::kUnknown;
  }
}

MultiRealsenseEmitterSynchronizerNode::MultiRealsenseEmitterSynchronizerNode(
  const rclcpp::NodeOptions & options)
: Node("realsense_sync_checker_node", options)
{
  // Default max sync tolerance is 1ms.
  constexpr int kMaxSyncToleranceUs = 1000;
  max_sync_tolerance_us_ = declare_parameter<int>("max_sync_tolerance_us", kMaxSyncToleranceUs);
  if (max_sync_tolerance_us_ <= 0) {
    const std::string error_message =
      "max_sync_tolerance_us parameter must be a positive integer.";
    RCLCPP_FATAL(get_logger(), "%s", error_message.c_str());
    throw std::invalid_argument(error_message);
  }

  camera_names_ = declare_parameter<std::vector<std::string>>(
    "camera_names", std::vector<std::string>{});

  if (camera_names_.empty()) {
    const std::string error_message =
      "camera_names parameter must be a list of camera names.";
    RCLCPP_FATAL(get_logger(), "%s", error_message.c_str());
    throw std::invalid_argument(error_message);
  }
  num_cameras_ = camera_names_.size();

  for (auto & camera_name : camera_names_) {
    camera_name.erase(0, camera_name.find_first_not_of('/'));
    camera_name.erase(camera_name.find_last_not_of('/') + 1);

    if (camera_name.empty()) {
      const std::string error_message =
        "camera_names parameter must be a list of non-empty camera names.";
      RCLCPP_FATAL(get_logger(), "%s", error_message.c_str());
      throw std::invalid_argument(error_message);
    }
  }

  metadata_subs_.reserve(num_cameras_);
  parameter_clients_.reserve(num_cameras_);
  emitter_syncs_.reserve(num_cameras_);

  for (int i = 0; i < num_cameras_; ++i) {
    const std::string topic_name = "/" + camera_names_[i] + "/infra1/metadata";
    metadata_subs_.emplace_back(std::make_unique<MetadataSub>(this, topic_name));
    parameter_clients_.emplace_back(
      std::make_shared<rclcpp::AsyncParametersClient>(this, "/" + camera_names_[i]));
    RCLCPP_INFO(
      get_logger(), "Subscribing to metadata topic %s, camera_name %s",
      topic_name.c_str(), camera_names_[i].c_str());
  }

  constexpr int kQueueSize = 10;
  if (num_cameras_ == 1) {
    metadata_subs_[0]->registerCallback(
      [this](MetadataMsg::ConstSharedPtr msg) {
        syncCallback1(msg);
      });
  } else if (num_cameras_ == 2) {
    auto synchronizer =
      std::make_unique<Synchronizer2>(
        SyncPolicy2(kQueueSize), *metadata_subs_[0], *metadata_subs_[1]);
    synchronizer->registerCallback(
      std::bind(
        &MultiRealsenseEmitterSynchronizerNode::syncCallback2, this, std::placeholders::_1,
        std::placeholders::_2));
    synchronizer_ = std::move(synchronizer);
  } else if (num_cameras_ == 3) {
    auto synchronizer = std::make_unique<Synchronizer3>(
      SyncPolicy3(kQueueSize), *metadata_subs_[0], *metadata_subs_[1], *metadata_subs_[2]);
    synchronizer->registerCallback(
      std::bind(
        &MultiRealsenseEmitterSynchronizerNode::syncCallback3, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3));
    synchronizer_ = std::move(synchronizer);
  } else if (num_cameras_ == 4) {
    auto synchronizer = std::make_unique<Synchronizer4>(
      SyncPolicy4(kQueueSize), *metadata_subs_[0], *metadata_subs_[1], *metadata_subs_[2],
      *metadata_subs_[3]);
    synchronizer->registerCallback(
      std::bind(
        &MultiRealsenseEmitterSynchronizerNode::syncCallback4, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    synchronizer_ = std::move(synchronizer);
  } else {
    RCLCPP_FATAL(get_logger(), "Unsupported number of cameras: %d", num_cameras_);
    throw std::invalid_argument("Unsupported number of cameras");
  }

  for (int i = 0; i < num_cameras_; ++i) {
    emitter_syncs_.emplace_back(
      std::make_unique<EmitterSynchronizer>(camera_names_[i], parameter_clients_[i]));
  }

  // Start timeout and exit if never receive synchronized messages.
  constexpr int kSyncStartTimeoutSec = 30;
  sync_start_timer_ = create_wall_timer(
    std::chrono::seconds(kSyncStartTimeoutSec),
    [this]() {
      if (!is_messages_synchronized_) {
        RCLCPP_ERROR(get_logger(),
          "Timed out waiting for synchronized messages after %d seconds. "
          "Exiting. Restart the program to try again.",
          kSyncStartTimeoutSec);
        rclcpp::shutdown();
      }
    });
}

void MultiRealsenseEmitterSynchronizerNode::syncCallback1(
  const MetadataMsg::ConstSharedPtr & msg1)
{
  syncCallback({msg1});
}

void MultiRealsenseEmitterSynchronizerNode::syncCallback2(
  const MetadataMsg::ConstSharedPtr & msg1,
  const MetadataMsg::ConstSharedPtr & msg2)
{
  syncCallback({msg1, msg2});
}

void MultiRealsenseEmitterSynchronizerNode::syncCallback3(
  const MetadataMsg::ConstSharedPtr & msg1,
  const MetadataMsg::ConstSharedPtr & msg2,
  const MetadataMsg::ConstSharedPtr & msg3)
{
  syncCallback({msg1, msg2, msg3});
}

void MultiRealsenseEmitterSynchronizerNode::syncCallback4(
  const MetadataMsg::ConstSharedPtr & msg1,
  const MetadataMsg::ConstSharedPtr & msg2,
  const MetadataMsg::ConstSharedPtr & msg3,
  const MetadataMsg::ConstSharedPtr & msg4)
{
  syncCallback({msg1, msg2, msg3, msg4});
}

bool MultiRealsenseEmitterSynchronizerNode::getCommonEmitterState(
  const std::vector<MetadataMsg::ConstSharedPtr> & msgs,
  std::vector<EmitterState> & emitter_states) const
{
  emitter_states.resize(num_cameras_, EmitterState::kUnknown);
  // Get common to decide the reference emitter state.
  int emitter_common_state = 0;
  for (int i = 0; i < num_cameras_; ++i) {
    EmitterState current_emitter_state = getEmitterStateFromMetadataMsg(msgs[i]);
    if (current_emitter_state == EmitterState::kUnknown) {
      RCLCPP_ERROR(
        get_logger(), "Camera %s has unknown emitter mode. Skipping",
        camera_names_[i].c_str());
      continue;
    }

    emitter_states[i] = current_emitter_state;
    if (current_emitter_state == EmitterState::kOn) {
      ++emitter_common_state;
    } else {
      --emitter_common_state;
    }
  }

  RCLCPP_DEBUG(get_logger(), "Emitter state common vote: %d", emitter_common_state);
  return emitter_common_state > 0;
}

bool MultiRealsenseEmitterSynchronizerNode::areMessagesSynchronized(
  const std::vector<MetadataMsg::ConstSharedPtr> & msgs)
{
  ++sync_count_;
  // In order to synchronize the camera emitter correctly, this needs synchronized messages
  // and no drops. So we will initially wait and check that the messages are in sync.
  rclcpp::Time first_timestamp(msgs[0]->header.stamp);
  for (size_t i = 1; i < msgs.size(); ++i) {
    rclcpp::Time current_timestamp(msgs[i]->header.stamp);
    const int64_t time_diff_ns =
      std::abs((current_timestamp - first_timestamp).nanoseconds());
    if (time_diff_ns > max_sync_tolerance_us_ * 1000) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 250,
        "Time difference between camera 0 and camera %zu is %ld ns, which is larger than the "
        "threshold of %d ns. Restarting sync check.",
        i, time_diff_ns, max_sync_tolerance_us_ * 1000);
      sync_count_ = 0;
    }
  }

  if (sync_count_ < kConsecutiveSyncsRequired) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Waiting for %d more frames for synchronization to stabilize.",
      kConsecutiveSyncsRequired - sync_count_);
    // If sync is lost while trying to synchronize emitter, depending on the emitter synchronizer
    // state it may fail to validate states and automatically reattempt.
    return false;
  }

  return true;
}

void MultiRealsenseEmitterSynchronizerNode::syncCallback(
  const std::vector<MetadataMsg::ConstSharedPtr> & msgs)
{
  RCLCPP_INFO_ONCE(
    get_logger(), "Received first approximately synchronized messages. Starting sync check.");

  if (!areMessagesSynchronized(msgs)) {
    return;
  }
  is_messages_synchronized_ = true;


  std::vector<EmitterState> emitter_states;
  const bool reference_emitter_state = getCommonEmitterState(msgs, emitter_states);

  // Synchronize the emitter state for each camera. Stop when all cameras are synchronized.
  bool sync_success = true;
  for (int i = 0; i < num_cameras_; ++i) {
    if (emitter_states[i] == EmitterState::kUnknown) {
      sync_success = false;
      continue;
    }

    sync_success &= emitter_syncs_[i]->syncEmitterMode(
      reference_emitter_state,
      static_cast<bool>(emitter_states[i]));
  }

  if (sync_success) {
    sync_complete_ = true;
    RCLCPP_INFO(
      get_logger(),
      "SUCCESS: All cameras synchronized emitter state in emitter_on_off mode!");
    rclcpp::shutdown();
  }
}
}  // namespace nvblox

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvblox::MultiRealsenseEmitterSynchronizerNode)
