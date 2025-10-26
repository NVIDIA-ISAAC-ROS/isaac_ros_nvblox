# Copyright (c) 2025, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 3.8)
project(nvblox_message_adapters)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -DLIBUS_USE_OPENSSL)
endif()
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ROS deps
find_package(rclcpp REQUIRED)
find_package(isaac_ros_common REQUIRED)
find_package(nvblox_msgs REQUIRED)

######## Main executable
add_executable(nvblox_voxel_layer_adapter
  src/nvblox_voxel_layer_adapter_node.cpp
  src/nvblox_voxel_layer_adapter_main.cpp
)
target_compile_features(nvblox_voxel_layer_adapter PUBLIC c_std_99 cxx_std_17)
target_include_directories(nvblox_voxel_layer_adapter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(nvblox_voxel_layer_adapter rclcpp isaac_ros_common nvblox_msgs)


######## Main executable for Mesh representation
add_executable(nvblox_mesh_layer_adapter
  src/nvblox_mesh_layer_adapter_node.cpp
  src/nvblox_mesh_layer_adapter_main.cpp
)
target_compile_features(nvblox_mesh_layer_adapter PUBLIC c_std_99 cxx_std_17)
target_include_directories(nvblox_mesh_layer_adapter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(nvblox_mesh_layer_adapter rclcpp isaac_ros_common nvblox_msgs)

######### Install & Export
include(GNUInstallDirs)
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS nvblox_voxel_layer_adapter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS nvblox_mesh_layer_adapter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE launch)
