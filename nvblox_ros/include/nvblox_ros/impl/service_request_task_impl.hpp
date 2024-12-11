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

#ifndef NVBLOX_ROS__IMPL__SERVICE_REQUEST_TASK_IMPL_HPP_
#define NVBLOX_ROS__IMPL__SERVICE_REQUEST_TASK_IMPL_HPP_

namespace nvblox
{

template<typename NodeType, typename ServiceType>
ServiceRequestTask<NodeType, ServiceType>::ServiceRequestTask(
  TaskFunctionType<NodeType, ServiceType> request_task, NodeType * node_ptr,
  RequestPtr<ServiceType> request_ptr, ResponsePtr<ServiceType> response_ptr)
: request_task_(request_task),
  node_ptr_(node_ptr),
  request_ptr_(request_ptr),
  response_ptr_(response_ptr) {}

template<typename NodeType, typename ServiceType>
bool ServiceRequestTask<NodeType, ServiceType>::executeTask()
{
  bool result = request_task_(node_ptr_, request_ptr_, response_ptr_);
  promise_.set_value();
  return result;
}

template<typename NodeType, typename ServiceType>
void ServiceRequestTask<NodeType, ServiceType>::waitForTaskCompletion()
{
  promise_.get_future().get();
}

}  // namespace nvblox

#endif  // NVBLOX_ROS__IMPL__SERVICE_REQUEST_TASK_IMPL_HPP_
