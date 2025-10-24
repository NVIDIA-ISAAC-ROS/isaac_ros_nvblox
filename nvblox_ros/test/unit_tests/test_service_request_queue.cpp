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

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <nvblox_msgs/srv/file_path.hpp>

#include "nvblox_ros/service_request_task.hpp"

namespace nvblox
{

class DummyNode {};

TEST(ServiceRequestTask, AsyncTest) {
  using ServiceType = nvblox_msgs::srv::FilePath;

  RequestPtr<ServiceType> request_pointer =
    std::make_shared<Request<ServiceType>>();
  ResponsePtr<ServiceType> response_pointer =
    std::make_shared<Response<ServiceType>>();
  DummyNode dummy_node;

  constexpr char kFilePath[] = "file.txt";
  request_pointer->file_path = kFilePath;

  // Define the task function
  TaskFunctionType<DummyNode, nvblox_msgs::srv::FilePath> request_task =
    [](DummyNode * node, RequestPtr<ServiceType> request,
    ResponsePtr<ServiceType> response) {
      (void)node;
      (void)request;
      // Wait for a bit to simulate this task being queued.
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      // Mark task as done.
      response->success = true;
      return response->success;
    };

  // Create the ServiceRequestTask
  ServiceRequestTask<DummyNode, nvblox_msgs::srv::FilePath> task(
    request_task, &dummy_node, request_pointer, response_pointer);

  // Launch the task execution asynchronously to simulate the task execution
  // running on a different thread that processes a task queue.
  auto f =
    std::async(
    std::launch::async,
    &ServiceRequestTask<DummyNode,
    nvblox_msgs::srv::FilePath>::executeTask,
    &task);

  // We should not have processed the task yet (didn't wait for completion).
  EXPECT_EQ(request_pointer->file_path, kFilePath);
  EXPECT_EQ(response_pointer->success, false);

  // Wait for the task to complete.
  task.waitForTaskCompletion();

  // Now we should get the result on the response pointer.
  EXPECT_EQ(request_pointer->file_path, kFilePath);
  EXPECT_EQ(response_pointer->success, true);
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
