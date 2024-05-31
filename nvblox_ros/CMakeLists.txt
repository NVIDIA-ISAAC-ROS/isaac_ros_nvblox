# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# Setup dependencies. Note that will also setup cuda architecture flags in CMAKE_CUDA_ARCHITECTURES
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Default behavior is to build for all supported CUDA architectures. In order to improve build
# speed we can select to build only for the native architecture by using
#    colcon build --cmake-args "-DUSE_NATIVE_CUDA_ARCHITECTURE=1"
# Then the core lib will determine the current cuda architecture for us.
if(DEFINED USE_NATIVE_CUDA_ARCHITECTURE)
  unset(CMAKE_CUDA_ARCHITECTURES)
  message(STATUS "nvblox_ros is built with native CUDA architectures.")
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  set(CMAKE_CUDA_ARCHITECTURES "87")
  message(STATUS "nvblox_ros is built for Jetson/Orin CUDA architecture: ${CMAKE_CUDA_ARCHITECTURES}")
else()
  message(STATUS "nvblox_ros is built for the following CUDA architectures: ${CMAKE_CUDA_ARCHITECTURES}")
endif()

# The core nvblox lib is built together with nvblox_ros
add_subdirectory(nvblox_core/nvblox)


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
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nvblox_ros_common REQUIRED)
find_package(isaac_ros_common REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(isaac_ros_managed_nitros REQUIRED)
find_package(isaac_ros_nitros_image_type REQUIRED)
find_package(isaac_ros_gxf REQUIRED)

##################
# COMPILER SETUP #
##################
include(nvblox_core/nvblox/cmake/setup_compilers.cmake)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}_lib SHARED
  src/lib/conversions/image_conversions.cpp
  src/lib/conversions/image_conversions_thrust.cu
  src/lib/conversions/layer_conversions.cpp
  src/lib/conversions/mesh_conversions.cpp
  src/lib/conversions/pointcloud_conversions.cu
  src/lib/conversions/esdf_slice_conversions.cu
  src/lib/conversions/esdf_and_gradients_conversions.cu
  src/lib/camera_cache.cpp
  src/lib/visualization.cpp
  src/lib/transformer.cpp
  src/lib/mapper_initialization.cpp
  src/lib/nvblox_node.cpp
  src/lib/nvblox_human_node.cpp
  src/lib/nvblox_node_params.cpp
)
target_link_libraries(${PROJECT_NAME}_lib nvblox_lib nvblox_eigen pthread)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nvblox_msgs
  std_msgs
  visualization_msgs
  std_srvs
  tf2_ros
  message_filters
  visualization_msgs
  cv_bridge
  nvblox_ros_common
  isaac_ros_managed_nitros
  isaac_ros_nitros_image_type
  isaac_ros_gxf
  isaac_ros_common
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME}_lib BEFORE PRIVATE
  $<TARGET_PROPERTY:nvblox_eigen,INTERFACE_INCLUDE_DIRECTORIES>)

rclcpp_components_register_nodes(${PROJECT_NAME}_lib "nvblox::NvbloxNode")
rclcpp_components_register_nodes(${PROJECT_NAME}_lib "nvblox::NvbloxHumanNode")

# NOTE(alexmillane 14.02.2024): This is needed at the moment to find libgxf_isaac_optimizer.so
# See: https://nvidia.slack.com/archives/C023NB2F7SN/p1707529762925559.
# TODO(alexmillane): Move to a generic Isaac ROS-wide solution at some point.
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE)

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18)
  get_target_property(CUDA_ARCHS nvblox_lib CUDA_ARCHITECTURES)
  set_property(TARGET ${PROJECT_NAME}_lib APPEND PROPERTY CUDA_ARCHITECTURES ${CUDA_ARCHS})
endif()

############
# BINARIES #
############
add_executable(nvblox_node
  src/nvblox_node_main.cpp
)
target_link_libraries(nvblox_node ${PROJECT_NAME}_lib)
set_target_properties(nvblox_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(nvblox_human_node
  src/nvblox_human_node_main.cpp
)
target_link_libraries(nvblox_human_node ${PROJECT_NAME}_lib)
set_target_properties(nvblox_human_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE)

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
  TARGETS ${PROJECT_NAME}_lib nvblox_lib nvblox_eigen nvblox_gpu_hash stdgpu
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

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}/scripts/
)

if(BUILD_TESTING)
  # Linting
  # NOTE(alexmillane): We can't use ament_lint_auto because the ignore paths are not fully respected in humble.
  #                    We therefore manually run the common linters we excludes configured where applicable.
  #                    This is fixed in later versions of ROS. See for example here: https://github.com/ament/ament_lint/issues/423
  #                    Once we move to a more recent version of ROS put the lines below back in.
  # find_package(ament_lint_auto REQUIRED)
  # Exclude submodule from linting tests
  # file(GLOB_RECURSE AMENT_LINT_AUTO_FILE_EXCLUDE
  #   nvblox_core/*
  # )
  # ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright("include" "scripts" "src" "test")
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck("include" "src" "test")
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint("include" "src" "test")
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8("test")
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake("CMakeLists.txt" "test")
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify("include" "src" "test")
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

  # The FindPythonInterp and FindPythonLibs modules are removed
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
  endif()

  # Integrations tests
  add_subdirectory(test/integration_tests)

  # Unit Tests
  add_subdirectory(test/unit_tests)
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
