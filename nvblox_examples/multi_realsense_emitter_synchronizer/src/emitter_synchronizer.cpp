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

#include "multi_realsense_emitter_synchronizer/emitter_synchronizer.hpp"

#include <future>
#include <vector>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace nvblox
{

template<typename T>
void EmitterSynchronizer::setParameter(const std::string & name, const T & value)
{
  if (!parameter_client_->service_is_ready()) {
    RCLCPP_ERROR(
      logger_, "Param service not ready for '%s', skipping set (%s)",
      camera_name_.c_str(), name.c_str());
    return;
  }

  parameter_client_->set_parameters(
    {rclcpp::Parameter(name, value)},
    [logger = logger_, camera_name = camera_name_, name]
    (std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f) {
      try {
        for (const auto & result : f.get()) {
          if (result.successful) {
            RCLCPP_INFO(
              logger, "Successfully set parameter on %s",
              camera_name.c_str());
          } else {
            RCLCPP_ERROR(logger, "Failed to set parameter on %s: %s",
                         camera_name.c_str(), result.reason.c_str());
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          logger, "Set exception on %s for %s: %s",
          camera_name.c_str(), name.c_str(), e.what());
      }
    });
}

void EmitterSynchronizer::checkParameter(
  const std::string & name, const std::string & expected_value)
{
  if (!parameter_client_->service_is_ready()) {
    RCLCPP_ERROR(
      logger_, "Param service not ready for '%s', skipping check (%s)",
      camera_name_.c_str(), name.c_str());
    return;
  }

  parameter_client_->get_parameters(
    {name}, [logger = logger_, camera_name = camera_name_, name, expected_value]
    (std::shared_future<std::vector<rclcpp::Parameter>> f) {
      try {
        auto parameters = f.get();
        if (parameters.empty()) {
          RCLCPP_WARN(
            logger, "Parameter '%s' not found on node '%s'",
            name.c_str(), camera_name.c_str());
          return;
        }
        const auto & param = parameters[0];
        if (param.value_to_string() != expected_value) {
          RCLCPP_WARN(
            logger, "Node %s paramater %s: expected '%s', got '%s'",
            camera_name.c_str(), name.c_str(), expected_value.c_str(),
            param.value_to_string().c_str());
        } else {
          RCLCPP_INFO(
            logger, "Node %s paramater %s is '%s' as expected",
            camera_name.c_str(), name.c_str(), param.value_to_string().c_str());
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(logger, "Error getting parameter: %s", e.what());
      }
    });
}

EmitterSynchronizer::EmitterSynchronizer(
  std::string camera_name,
  rclcpp::AsyncParametersClient::SharedPtr parameter_client)
: camera_name_(std::move(camera_name)),
  logger_(rclcpp::get_logger((camera_name_ + std::string("_emitter_syncer")).c_str())),
  parameter_client_(std::move(parameter_client))
{
  RCLCPP_INFO(logger_, "Checking parameters for camera %s", camera_name_.c_str());
  checkParameter("depth_module.emitter_enabled", "1");
}

bool EmitterSynchronizer::syncEmitterMode(
  const bool reference_emitter_state, const bool current_emitter_state)
{
  bool is_synced = false;
  constexpr int kMaxStateChangeCountBeforeRetry = 30;
  constexpr int kConsecutiveSyncsRequired = 10;

  auto setEmitterOnOff = [this](bool enable) {
      RCLCPP_INFO(
        logger_, "Setting emitter_on_off to '%s' for camera %s",
        enable ? "true" : "false", camera_name_.c_str());
      setParameter("depth_module.emitter_on_off", enable);
    };

  switch (state_) {
    case State::RESETTING:
      if (state_check_count_ == 0) {
        setEmitterOnOff(false);
        ++state_check_count_;
        break;
      }
        // When emitter_on_off mode is disabled, emitter state is then consistent on.
        // This assumes the camera emitter_enabled is true.
      if (current_emitter_state && prev_emitter_state_) {
        ++valid_state_count_;
        if (valid_state_count_ >= kConsecutiveSyncsRequired) {
          RCLCPP_INFO(
            logger_,
            "Camera %s disable emitter_on_off complete after %d frames. Re-enabling...",
            camera_name_.c_str(), state_check_count_);
          state_ = State::INITIALIZING;
          valid_state_count_ = 0;
          state_check_count_ = 0;
          break;
        }
      }

      if (state_check_count_ <= kMaxStateChangeCountBeforeRetry) {
        ++state_check_count_;
      } else {
        RCLCPP_ERROR(
          logger_, "Camera %s did not disable emitter_on_off mode, retrying.",
          camera_name_.c_str());
        resetSyncAttempt();
      }
      break;
    case State::INITIALIZING:
      // In this state, we just set and wait until the camera changes mode.
      if (state_check_count_ == 0) {
        RCLCPP_INFO(
          logger_, "Starting emitter synchronization for camera %s.",
          camera_name_.c_str());
        setEmitterOnOff(true);
        ++state_check_count_;
        break;
      }
      // When the initial state changes, should mean camera transitioned into emitter_on_off mode
      // from initially off.
      if (current_emitter_state == !prev_emitter_state_) {
        RCLCPP_INFO(
          logger_, "Camera %s enable emitter_on_off mode complete after %d frames",
          camera_name_.c_str(), state_check_count_);
        checkParameter("depth_module.emitter_on_off", "true");
        state_ = State::VALIDATING;
        valid_state_count_ = 0;
        state_check_count_ = 0;
        break;
      }

      if (state_check_count_ <= kMaxStateChangeCountBeforeRetry) {
        ++state_check_count_;
      } else {
        RCLCPP_ERROR(
          logger_, "Camera %s did not enable emitter_on_off mode, retrying.",
          camera_name_.c_str());
        resetSyncAttempt();
      }
      break;
    case State::VALIDATING:
      // Emitter state in emitter_on_off mode should toggle between on and off each frame,
      // so the reference emitter state is the opposite of the previous emitter state.
      if (current_emitter_state == !prev_emitter_state_) {
        RCLCPP_DEBUG(logger_, "Camera %s toggle check %d/%d",
          camera_name_.c_str(), valid_state_count_, kConsecutiveSyncsRequired);
        ++valid_state_count_;
        if (valid_state_count_ >= kConsecutiveSyncsRequired) {
          RCLCPP_INFO(
            logger_, "Camera %s validated is running in emitter_on_off mode",
            camera_name_.c_str());
          state_ = State::SYNC_CHECKING;
          valid_state_count_ = 0;
          state_check_count_ = 0;
          break;
        }
      }

      if (state_check_count_ <= kMaxStateChangeCountBeforeRetry) {
        ++state_check_count_;
      } else {
        RCLCPP_ERROR(
          logger_, "Camera %s emitter state is not in emitter_on_off mode, retrying.",
          camera_name_.c_str());
        resetSyncAttempt();
      }
      break;
    case State::SYNC_CHECKING:
      if (reference_emitter_state == current_emitter_state) {
        RCLCPP_DEBUG(logger_, "Camera %s aligned with reference state %d - check %d/%d",
          camera_name_.c_str(), reference_emitter_state, valid_state_count_,
          kConsecutiveSyncsRequired);
        ++valid_state_count_;
        if (valid_state_count_ >= kConsecutiveSyncsRequired) {
          RCLCPP_INFO(logger_, "Camera %s successfully synchronized!", camera_name_.c_str());
          state_ = State::SYNCHRONIZED;
          is_synced = true;
        }
      } else {
        // We won't always align the cameras after enabling emitter_on_off, so some retries
        // are expected.
        RCLCPP_WARN(
          logger_, "Camera %s is not the expected emitter state (%d != %d), retrying.",
          camera_name_.c_str(), reference_emitter_state, current_emitter_state);
        resetSyncAttempt();
      }
      break;
    case State::SYNCHRONIZED:
      if (reference_emitter_state == current_emitter_state) {
        is_synced = true;
      } else {
        RCLCPP_ERROR(
          logger_, "Camera %s lost synchronization (%d != %d), retrying.",
          camera_name_.c_str(), reference_emitter_state, current_emitter_state);
        resetSyncAttempt();
      }
      break;
    case State::FAILED:
      break;
  }

  prev_emitter_state_ = current_emitter_state;
  return is_synced;
}

void EmitterSynchronizer::resetSyncAttempt()
{
  ++attempts_;
  constexpr int kMaxResets = 10;
  if (attempts_ <= kMaxResets) {
    state_ = State::RESETTING;
    state_check_count_ = 0;
    valid_state_count_ = 0;
  } else {
    RCLCPP_ERROR(
      logger_, "FAILURE: Camera %s has failed %d attempts, giving up.",
      camera_name_.c_str(), attempts_);
    state_ = State::FAILED;
  }
}

EmitterSynchronizer::State EmitterSynchronizer::getState() const
{
  return state_;
}

}  // namespace nvblox
