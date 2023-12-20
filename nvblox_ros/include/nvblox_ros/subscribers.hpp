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

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <memory>
#include <string>

namespace nvblox
{

/// Class that bundles an image/camera_info subscriber pair + a synchronizer
class ImageAndCameraInfoSyncedSubscriber
{
public:
  /// Time Sync policy
  typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>
    time_policy_t;

  /// Initialize the subscribers
  ///
  /// Subscribers will will Listen to topics:
  ///   {topic_basename}/image
  ///   {topic_basename}/camera_info
  ///
  /// @param parent           ROS2 Node that owns these subscribers
  /// @param qos              QOS for image and camera info topics
  /// @param topic_basename   Basename for topic
  /// @param callback         Will be called when synchronizer is triggered
  template<typename CallbackType>
  void subscribe(
    rclcpp::Node * parent, const rmw_qos_profile_t & qos,
    const std::string & topic_basename, CallbackType callback)
  {
    synchronizer_.reset();

    image_.subscribe(parent, topic_basename + "/image", qos);
    camera_info_.subscribe(parent, topic_basename + "/camera_info", qos);

    synchronizer_ =
      std::make_unique<message_filters::Synchronizer<time_policy_t>>(
      time_policy_t(kQueueSize), image_, camera_info_);
    synchronizer_->registerCallback(callback);
  }

  /// Remove the synchronizer callback
  void resetSynchronizer() {synchronizer_.reset();}

  message_filters::Subscriber<sensor_msgs::msg::Image> & getImageSubscriber()
  {
    return image_;
  }
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> &
  getCameraInfoSubscriber()
  {
    return camera_info_;
  }

private:
  static constexpr int kQueueSize = 10;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_;
  std::unique_ptr<message_filters::Synchronizer<time_policy_t>> synchronizer_;
};

}  // namespace nvblox
