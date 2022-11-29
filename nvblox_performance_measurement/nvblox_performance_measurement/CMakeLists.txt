# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(nvblox_performance_measurement)
project(nvblox_performance_measurement LANGUAGES CXX CUDA)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nvblox_msgs REQUIRED)
find_package(nvblox_ros REQUIRED)
find_package(nvblox_performance_measurement_msgs REQUIRED)
find_package(CUDA REQUIRED) # Should probably be replaced with CUDAToolkit when available.

# include
include_directories(include)

# libraries

# executables
add_executable(results_collector_node
  src/results_collector_node_main.cpp
  src/lib/message_stamp_recorder.cpp
  src/lib/results_collector_node.cpp
)
ament_target_dependencies(results_collector_node
  rclcpp
  sensor_msgs
  std_srvs
  nvblox_msgs
  nvblox_ros
  nvblox_performance_measurement_msgs
) 

add_executable(nvblox_performance_measurement_node
  src/nvblox_performance_measurement_node_main.cpp
  src/lib/nvblox_performance_measurement_node.cpp
)
ament_target_dependencies(nvblox_performance_measurement_node
  nvblox_ros
  std_msgs
  nvblox_performance_measurement_msgs
  CUDA
)

# install
install(TARGETS
  results_collector_node
  nvblox_performance_measurement_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install python scripts
install(PROGRAMS
  scripts/run_benchmark.py
  scripts/analyze_timestamps.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
