# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
project(odometry_flattener)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

#############
# LIBRARIES #
#############
add_library(odometry_flattener_component
  src/odometry_flattener_node.cpp
)
target_compile_definitions(odometry_flattener_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(odometry_flattener_component
  rclcpp
  rclcpp_components
  tf2_ros
  tf2_msgs
  tf2_eigen
)
target_include_directories(odometry_flattener_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_nodes(odometry_flattener_component "nvblox::OdometryFlattenerNode")
set(node_plugins "${node_plugins}nvblox::OdometryFlattenerNode;$<TARGET_FILE:odometry_flattener_component>\n")

############
# BINARIES #
############
add_executable(odometry_flattener_node
  src/odometry_flattener_node_main.cpp
)
target_link_libraries(odometry_flattener_node odometry_flattener_component)


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
  TARGETS odometry_flattener_component
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# nodes
install(
  TARGETS odometry_flattener_node
  DESTINATION lib/${PROJECT_NAME}
)

##########
# EXPORT #
##########
ament_export_include_directories(
  include
)
ament_export_libraries(
  odometry_flattener_component
)
ament_export_targets(
  ${PROJECT_NAME}Targets
)

ament_export_dependencies(
  rclcpp
  tf2_ros
  tf2_msgs
  tf2_eigen
)

ament_package()
