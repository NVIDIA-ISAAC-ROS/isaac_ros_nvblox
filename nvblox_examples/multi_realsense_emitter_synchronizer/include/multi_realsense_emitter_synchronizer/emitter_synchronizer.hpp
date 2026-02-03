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

#ifndef MULTI_REALSENSE_EMITTER_SYNCHRONIZER__EMITTER_SYNCHRONIZER_HPP_
#define MULTI_REALSENSE_EMITTER_SYNCHRONIZER__EMITTER_SYNCHRONIZER_HPP_

#include <memory>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/parameter_client.hpp>

namespace nvblox
{

class EmitterSynchronizer
{
public:
  enum class State
  {
    RESETTING,
    INITIALIZING,
    VALIDATING,
    SYNC_CHECKING,
    SYNCHRONIZED,
    FAILED
  };

  explicit EmitterSynchronizer(
    std::string camera_name,
    rclcpp::AsyncParametersClient::SharedPtr parameter_client);

  bool syncEmitterMode(const bool reference_emitter_state, const bool current_emitter_state);
  void resetSyncAttempt();
  State getState() const;

private:
  template<typename T>
  void setParameter(const std::string & name, const T & value);
  void checkParameter(const std::string & name, const std::string & expected_value);

  std::string camera_name_;
  rclcpp::Logger logger_;
  rclcpp::AsyncParametersClient::SharedPtr parameter_client_;
  State state_ = State::INITIALIZING;
  bool prev_emitter_state_ = true;
  int state_check_count_ = 0;
  int valid_state_count_ = 0;
  int attempts_ = 0;
};

}  // namespace nvblox

#endif  // MULTI_REALSENSE_EMITTER_SYNCHRONIZER__EMITTER_SYNCHRONIZER_HPP_
