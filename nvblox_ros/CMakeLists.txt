# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nvblox_msgs REQUIRED)
find_package(libstatistics_collector REQUIRED)
find_package(message_filters REQUIRED)
find_package(Threads REQUIRED)
find_package(nvblox REQUIRED)
find_package(glog REQUIRED)
find_package(cuda_image_convert REQUIRED)

########
# CUDA #
########
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr -Xcudafe --display_error_number --disable-warnings ")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --compiler-options -fPIC")

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}_lib SHARED
  src/lib/conversions.cpp
  src/lib/cuda/conversions.cu
  src/lib/transformer.cpp
  src/lib/nvblox_node.cpp
)
target_link_libraries(${PROJECT_NAME}_lib nvblox::nvblox_lib nvblox::nvblox_eigen cuda_image_convert)
ament_target_dependencies(${PROJECT_NAME}_lib
  nvblox
  sensor_msgs
  geometry_msgs
  nvblox_msgs
  rclcpp
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  message_filters
  cuda_image_convert
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME}_lib BEFORE PRIVATE
  $<TARGET_PROPERTY:nvblox::nvblox_eigen,INTERFACE_INCLUDE_DIRECTORIES>)

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18)
  get_target_property(CUDA_ARCHS nvblox::nvblox_lib CUDA_ARCHITECTURES)
  set_target_properties(${PROJECT_NAME}_lib PROPERTIES CUDA_ARCHITECTURES ${CUDA_ARCHS})
endif()

############
# BINARIES #
############
add_executable(nvblox_node
  src/nvblox_node.cpp
)
target_link_libraries(nvblox_node ${PROJECT_NAME}_lib)

add_executable(3dmatch_node
  src/3dmatch_node.cpp
)
target_link_libraries(3dmatch_node ${PROJECT_NAME}_lib)

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
  TARGETS nvblox_node 3dmatch_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Ignore copyright notices since we use custom NVIDIA Isaac ROS Software License
  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
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
)

ament_package()
