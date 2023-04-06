cmake_minimum_required(VERSION 3.5)
project(nvblox_ros_common)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}_lib SHARED
  src/qos.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

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
)

ament_package()
