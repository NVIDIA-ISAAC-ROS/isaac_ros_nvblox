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

cmake_minimum_required(VERSION 3.10)

# This is 100% not the right thing to do. Requies cmake magic to find the exiting cuda compiler.
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

project(nvblox_ros LANGUAGES CXX CUDA)
add_compile_options(-Wall -Wextra -Wpedantic -O3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()


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


list(APPEND CMAKE_INCLUDE_PATH "/usr/local/include")

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(BoldMagenta "${Esc}[1;35m")
  set(Magenta     "${Esc}[35m")
endif()

message("${Magenta}INFO: ${CMAKE_MODULE_PATH}.${ColourReset}")
message("${Magenta}INFO: ${CMAKE_PREFIX_PATH}.${ColourReset}")
message("${Magenta}INFO: ${CMAKE_FIND_ROOT_PATH}.${ColourReset}")
message("${Magenta}INFO: ${CMAKE_INCLUDE_PATH}.${ColourReset}")

find_package (Eigen3 3.3.7 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  nvblox_msgs
  cv_bridge
  message_filters
  Threads
  #nvblox_ros_common
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_eigen
    nvblox_msgs
    Threads
    message_filters
    visualization_msgs
)

# Process nvblox
set(NVBLOX_DIR ../nvblox/nvblox)

# Disable tests and experiments
set(BUILD_EXPERIMENTS OFF)
set(BUILD_TESTS OFF)

# Direct the child cmake to install eigen to the devel space.
# Note that we put it in the include/nvblox_ros directory, which should stop it
# colliding with other versions of eigen used by other catkin packages.
set(NVBLOX_EIGEN_CATKIN_INCLUDE_DESTINATION "${CMAKE_INSTALL_PREFIX}")
set(EIGEN_INCLUDE_DESTINATION "${NVBLOX_EIGEN_CATKIN_INCLUDE_DESTINATION}")
set(STDGPU_INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX})

##########
# NVBLOX #
##########
add_subdirectory(${NVBLOX_DIR} nvblox)

include_directories(AFTER include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

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
  ros
  ros_components
  nvblox
  sensor_msgs
  geometry_msgs
  nvblox_msgs
  ros
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

ros_components_register_nodes(${PROJECT_NAME}_lib "nvblox::NvbloxNode")
ros_components_register_nodes(${PROJECT_NAME}_lib "nvblox::NvbloxHumanNode")


target_link_libraries(${PROJECT_NAME}_lib nvblox::nvblox_lib nvblox::nvblox_eigen pthread
  nvblox_interface
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_dependencies(${PROJECT_NAME}_lib
  ${catkin_EXPORTED_TARGETS})

target_include_directories(${PROJECT_NAME}_lib SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS})  



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

add_dependencies(nvblox_node
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(nvblox_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

add_executable(nvblox_human_node
  src/nvblox_human_node_main.cpp
)
target_link_libraries(nvblox_human_node ${PROJECT_NAME}_lib)

add_dependencies(nvblox_human_node
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(nvblox_human_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

# 62 Should be TX2, jetson
set_target_properties(${PROJECT_NAME}_lib PROPERTIES CUDA_ARCHITECTURES "62;72;75;80;86;87")

###########
# INSTALL #
###########
# catkin_simple doesn't manage everything in our case, where we're installing
# not only the ros package nvblox_ros, but it's dependancies from nvblox.
# We therefore export the nvblox target along with the nvblox_ros stuff.
get_target_property(nvblox_INCLUDES nvblox_interface INTERFACE_INCLUDE_DIRECTORIES)

install(
  DIRECTORY 
    include/${PROJECT_NAME}/
  DESTINATION include
)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_lib nvblox_lib nvblox_gpu_hash nvblox_cuda_check
  EXPORT ${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install nodes
install(
  TARGETS nvblox_node nvblox_human_node
  DESTINATION lib/${PROJECT_NAME}
)
