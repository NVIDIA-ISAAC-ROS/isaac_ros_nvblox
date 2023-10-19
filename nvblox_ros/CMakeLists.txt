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

cmake_minimum_required(VERSION 3.10)
project(nvblox_ros LANGUAGES CXX CUDA)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default to release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

################
# DEPENDENCIES #
################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nvblox_msgs REQUIRED)
find_package(libstatistics_collector REQUIRED)
find_package(message_filters REQUIRED)
find_package(Threads REQUIRED)
find_package(nvblox REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nvblox_ros_common REQUIRED)
find_package(CUDAToolkit REQUIRED)

########
# CUDA #
########
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr -Xcudafe --display_error_number --disable-warnings ")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --compiler-options -fPIC")

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}_lib SHARED
  src/lib/conversions/image_conversions.cu
  src/lib/conversions/layer_conversions.cu
  src/lib/conversions/mesh_conversions.cpp
  src/lib/conversions/pointcloud_conversions.cu
  src/lib/conversions/esdf_slice_conversions.cu
  src/lib/visualization.cpp
  src/lib/transformer.cpp
  src/lib/mapper_initialization.cpp
  src/lib/nvblox_node.cpp
  src/lib/nvblox_human_node.cpp
)
target_link_libraries(${PROJECT_NAME}_lib nvblox::nvblox_lib nvblox::nvblox_eigen pthread)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  nvblox
  sensor_msgs
  geometry_msgs
  nvblox_msgs
  rclcpp
  std_msgs
  visualization_msgs
  std_srvs
  tf2_ros
  message_filters
  visualization_msgs
  cv_bridge
  nvblox_ros_common
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME}_lib BEFORE PRIVATE
  $<TARGET_PROPERTY:nvblox::nvblox_eigen,INTERFACE_INCLUDE_DIRECTORIES>)

rclcpp_components_register_nodes(${PROJECT_NAME}_lib "nvblox::NvbloxNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_lib "nvblox::NvbloxHumanNode")

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18)
  get_target_property(CUDA_ARCHS nvblox::nvblox_lib CUDA_ARCHITECTURES)
  set_property(TARGET ${PROJECT_NAME}_lib APPEND PROPERTY CUDA_ARCHITECTURES ${CUDA_ARCHS})
endif()

############
# BINARIES #
############
add_executable(nvblox_node
  src/nvblox_node_main.cpp
)
target_link_libraries(nvblox_node ${PROJECT_NAME}_lib)

add_executable(nvblox_human_node
  src/nvblox_human_node_main.cpp
)
target_link_libraries(nvblox_human_node ${PROJECT_NAME}_lib)

###########
# INSTALL #
###########
# Install includes.
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the library target.
install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install nodes which live in lib for some reason.
install(
  TARGETS nvblox_node nvblox_human_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install tests
install(DIRECTORY
  test
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()


  # The FindPythonInterp and FindPythonLibs modules are removed
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_nvblox_pol.py)
endif()

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
  nvblox
  tf2_ros
  message_filters
  libstatistics_collector
  visualization_msgs
  cv_bridge
  nvblox_ros_common
)

ament_package()
