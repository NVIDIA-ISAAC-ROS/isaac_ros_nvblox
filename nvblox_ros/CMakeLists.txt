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

# Process nvblox
set(NVBLOX_DIR ../nvblox/nvblox)

# Disable tests and experiments
set(BUILD_EXPERIMENTS OFF)
set(BUILD_TESTS OFF)

# Direct the child cmake to install eigen to the devel space.
# Note that we put it in the include/nvblox_ros directory, which should stop it
# colliding with other versions of eigen used by other catkin packages.
set(NVBLOX_EIGEN_AMENT_INCLUDE_DESTINATION "${CMAKE_INSTALL_PREFIX}")
set(EIGEN_INCLUDE_DESTINATION "${NVBLOX_EIGEN_AMENT_INCLUDE_DESTINATION}")
set(STDGPU_INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX})

########
# CUDA #
########
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr -Xcudafe --display_error_number --disable-warnings ")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --compiler-options -fPIC")

##########
# NVBLOX #
##########
add_subdirectory(${NVBLOX_DIR} nvblox)

include_directories(AFTER include)


#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}_lib SHARED
  src/conversions.cpp
  src/cuda/conversions.cu
  src/transformer.cpp
)
target_link_libraries(${PROJECT_NAME}_lib nvblox_interface)
ament_target_dependencies(${PROJECT_NAME}_lib
    sensor_msgs
    geometry_msgs
    nvblox_msgs
)

############
# BINARIES #
############
add_executable(nvblox_node
  src/nvblox_node.cpp
)
target_link_libraries(nvblox_node ${PROJECT_NAME}_lib)
ament_target_dependencies(nvblox_node rclcpp std_msgs std_srvs sensor_msgs tf2_ros tf2_eigen)

add_executable(3dmatch_node
  src/3dmatch_node.cpp
)
target_link_libraries(3dmatch_node ${PROJECT_NAME}_lib)
ament_target_dependencies(3dmatch_node rclcpp std_msgs sensor_msgs tf2_ros tf2_eigen)


###########
# INSTALL #
###########
# catkin_simple doesn't manage everything in our case, where we're installing
# not only the ros package nvblox_ros, but it's dependancies from nvblox.
# We therefore export the nvblox target along with the nvblox_ros stuff.
get_target_property(nvblox_INCLUDES nvblox_interface INTERFACE_INCLUDE_DIRECTORIES)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_lib nvblox_lib nvblox_gpu_hash nvblox_cuda_check
  EXPORT nvbloxTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install nodes
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
ament_package()
