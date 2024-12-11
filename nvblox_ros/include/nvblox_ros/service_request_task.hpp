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

#ifndef NVBLOX_ROS__SERVICE_REQUEST_TASK_HPP_
#define NVBLOX_ROS__SERVICE_REQUEST_TASK_HPP_

#include <functional>
#include <future>
#include <memory>

namespace nvblox
{

template<typename ServiceType>
using Request = typename ServiceType::Request;

template<typename ServiceType>
using Response = typename ServiceType::Response;

template<typename ServiceType>
using RequestPtr = std::shared_ptr<Request<ServiceType>>;

template<typename ServiceType>
using ResponsePtr = std::shared_ptr<Response<ServiceType>>;

template<typename NodeType, typename ServiceType>
using TaskFunctionType = std::function<bool (
      NodeType *, RequestPtr<ServiceType>, ResponsePtr<ServiceType>)>;


/// @brief A struct defining a task requested by a service call.
/// @tparam NodeType The type of the ROS service node.
/// @tparam ServiceType The type of the service.
template<typename NodeType, typename ServiceType>
struct ServiceRequestTask
{
  /// @brief Constructor of the ServiceRequestTask.
  /// @param request_task The task function that should be processed for the service call.
  /// @param node_ptr Pointer to the ROS service node that received the service call.
  /// @param request_ptr Pointer to the request argument of the service.
  /// @param response_ptr Pointer to the response_ptr of the service.
  ServiceRequestTask(
    TaskFunctionType<NodeType, ServiceType> request_task,
    NodeType * node_ptr, RequestPtr<ServiceType> request_ptr,
    ResponsePtr<ServiceType> response_ptr);

  /// @brief Executing the request task passed to the constructor and mark it complete.
  /// @return The boolean result returned by the request task.
  bool executeTask();

  /// @brief Waiting until the request task is completed (on executeTask call).
  void waitForTaskCompletion();

  // Promise that is set when request task is finished.
  std::promise<void> promise_;

  TaskFunctionType<NodeType, ServiceType> request_task_;
  NodeType * node_ptr_;
  RequestPtr<ServiceType> request_ptr_;
  ResponsePtr<ServiceType> response_ptr_;
};

}  // namespace nvblox

#include "nvblox_ros/impl/service_request_task_impl.hpp"

#endif  // NVBLOX_ROS__SERVICE_REQUEST_TASK_HPP_
