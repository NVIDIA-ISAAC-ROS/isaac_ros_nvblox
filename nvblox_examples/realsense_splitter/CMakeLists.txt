# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
project(realsense_splitter)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(nvblox_ros_common REQUIRED)
find_package(CUDAToolkit REQUIRED)


#############
# LIBRARIES #
#############
add_library(realsense_splitter_component
  src/realsense_splitter_node.cpp
)
target_compile_definitions(realsense_splitter_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(realsense_splitter_component
  rclcpp
  rclcpp_components
  sensor_msgs
  message_filters
  realsense2_camera_msgs
  nvblox_ros_common
)
target_include_directories(realsense_splitter_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_nodes(realsense_splitter_component "nvblox::RealsenseSplitterNode")
set(node_plugins "${node_plugins}nvblox::RealsenseSplitterNode;$<TARGET_FILE:realsense_splitter_component>\n")


############
# BINARIES #
############
add_executable(realsense_splitter_node
  src/realsense_splitter_node_main.cpp
)
target_link_libraries(realsense_splitter_node realsense_splitter_component)

###########
# INSTALL #
###########

# includes
install(
  DIRECTORY include/
  DESTINATION include
)

# libs
install(
  TARGETS realsense_splitter_component
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# nodes
install(TARGETS realsense_splitter_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

##########
# EXPORT #
##########
ament_export_include_directories(
  include
)
ament_export_libraries(
  realsense_splitter_component
)
ament_export_targets(
  ${PROJECT_NAME}Targets
)

ament_export_dependencies(
  rclcpp
  sensor_msgs
)

ament_package()
