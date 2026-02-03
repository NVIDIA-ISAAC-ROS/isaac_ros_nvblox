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

#ifndef MULTI_REALSENSE_EMITTER_SYNCHRONIZER__MULTI_REALSENSE_EMITTER_SYNCHRONIZER_NODE_HPP_
#define MULTI_REALSENSE_EMITTER_SYNCHRONIZER__MULTI_REALSENSE_EMITTER_SYNCHRONIZER_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <memory>
#include <string>
#include <variant>
#include <vector>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realsense2_camera_msgs/msg/metadata.hpp>

#include "multi_realsense_emitter_synchronizer/emitter_synchronizer.hpp"

namespace nvblox
{

class MultiRealsenseEmitterSynchronizerNode : public rclcpp::Node
{
public:
  enum class EmitterState : int
  {
    kOff = 0,
    kOn = 1,
    kUnknown = 2
  };
  explicit MultiRealsenseEmitterSynchronizerNode(const rclcpp::NodeOptions & options);
  virtual ~MultiRealsenseEmitterSynchronizerNode() = default;

  // Returns true if emitter synchronization completed successfully.
  bool isSyncComplete() const {return sync_complete_;}

private:
  using MetadataMsg = realsense2_camera_msgs::msg::Metadata;
  using MetadataSub = message_filters::Subscriber<MetadataMsg>;
  using SyncPolicy2 = message_filters::sync_policies::ApproximateTime<
    MetadataMsg, MetadataMsg>;
  using SyncPolicy3 = message_filters::sync_policies::ApproximateTime<
    MetadataMsg, MetadataMsg, MetadataMsg>;
  using SyncPolicy4 = message_filters::sync_policies::ApproximateTime<
    MetadataMsg, MetadataMsg, MetadataMsg, MetadataMsg>;
  using Synchronizer2 = message_filters::Synchronizer<SyncPolicy2>;
  using Synchronizer3 = message_filters::Synchronizer<SyncPolicy3>;
  using Synchronizer4 = message_filters::Synchronizer<SyncPolicy4>;

  void syncCallback(const std::vector<MetadataMsg::ConstSharedPtr> & msgs);
  void syncCallback1(
    const MetadataMsg::ConstSharedPtr & msg1);
  void syncCallback2(
    const MetadataMsg::ConstSharedPtr & msg1,
    const MetadataMsg::ConstSharedPtr & msg2);
  void syncCallback3(
    const MetadataMsg::ConstSharedPtr & msg1,
    const MetadataMsg::ConstSharedPtr & msg2,
    const MetadataMsg::ConstSharedPtr & msg3);
  void syncCallback4(
    const MetadataMsg::ConstSharedPtr & msg1,
    const MetadataMsg::ConstSharedPtr & msg2,
    const MetadataMsg::ConstSharedPtr & msg3,
    const MetadataMsg::ConstSharedPtr & msg4);

  EmitterState getEmitterStateFromMetadataMsg(const MetadataMsg::ConstSharedPtr & msg) const;
  bool areMessagesSynchronized(const std::vector<MetadataMsg::ConstSharedPtr> & msgs);
  bool getCommonEmitterState(
    const std::vector<MetadataMsg::ConstSharedPtr> & msgs,
    std::vector<EmitterState> & emitter_states) const;

  int num_cameras_;
  int max_sync_tolerance_us_;
  std::vector<std::string> camera_names_;
  int sync_count_ = 0;
  bool is_messages_synchronized_ = false;
  rclcpp::TimerBase::SharedPtr sync_start_timer_;

  std::vector<std::unique_ptr<MetadataSub>> metadata_subs_;
  std::variant<std::unique_ptr<Synchronizer2>, std::unique_ptr<Synchronizer3>,
    std::unique_ptr<Synchronizer4>>
  synchronizer_;

  std::vector<rclcpp::AsyncParametersClient::SharedPtr> parameter_clients_;
  std::vector<std::unique_ptr<EmitterSynchronizer>> emitter_syncs_;

  // At 60FPS, 5 seconds is 300 frames.
  static constexpr int kConsecutiveSyncsRequired = 300;

  bool sync_complete_ = false;
};

}  // namespace nvblox

#endif  // MULTI_REALSENSE_EMITTER_SYNCHRONIZER__MULTI_REALSENSE_EMITTER_SYNCHRONIZER_NODE_HPP_
