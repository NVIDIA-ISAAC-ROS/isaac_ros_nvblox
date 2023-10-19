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
project(nvblox_image_padding)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nvblox_ros_common REQUIRED)
find_package(CUDAToolkit REQUIRED)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}_lib SHARED
  src/image_padding_cropping_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  nvblox_ros_common
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_nodes(${PROJECT_NAME}_lib "nvblox::ImagePaddingCroppingNode")
  
############
# BINARIES #
############
add_executable(image_padding_cropping_node
  src/image_padding_cropping_node_main.cpp
)
target_link_libraries(image_padding_cropping_node ${PROJECT_NAME}_lib)

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
  TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# nodes
install(
  TARGETS image_padding_cropping_node
  DESTINATION lib/${PROJECT_NAME}
)

##########
# EXPORT #
##########
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}_lib
)
ament_export_targets(
  ${PROJECT_NAME}Targets
)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
)

ament_package()
