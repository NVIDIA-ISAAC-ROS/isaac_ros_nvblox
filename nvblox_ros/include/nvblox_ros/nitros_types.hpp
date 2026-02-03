// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVBLOX_ROS__NITROS_TYPES_HPP_
#define NVBLOX_ROS__NITROS_TYPES_HPP_

#include <memory>
#include <string>
#include <utility>

#include <isaac_ros_managed_nitros/managed_nitros_subscriber.hpp>
#include <isaac_ros_nitros_image_type/nitros_image_view.hpp>

namespace nvblox
{
/// The message type transmitted by nitros
using NitrosView = nvidia::isaac_ros::nitros::NitrosImageView;

/// Nitros Subscriber type
using NitrosViewSubscriber =
  nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<NitrosView>;

/// Need shared pointer when buffering the messages, since the raw NitrosImageView does not support
/// erase()
using NitrosViewPtr = std::shared_ptr<NitrosView>;

// TODO(dtingdahl) the Nitros messages only store a weak message to the underlying ROS message
// which can be invalidated at any time. We therefore store the frame_id separately to avoid
// problems when buffering our messages. We can remove this struct once this is fixed in Nitros.
using NitrosViewPtrAndFrameId = std::pair<NitrosViewPtr, std::string>;
}  // namespace nvblox

#endif  // NVBLOX_ROS__NITROS_TYPES_HPP_"
