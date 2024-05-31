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

find_package(ament_cmake_gtest REQUIRED)

function(add_nvblox_ros_unit_test test_name)
    # Switch between .cpp and .cu tests.
    set(test_file_extension "cpp")
    set(is_cuda_test ${ARGN})
    if(${is_cuda_test})
        set(test_file_extension "cu")
    endif()
    # Build the test
    ament_add_gtest(${test_name} ${test_name}.${test_file_extension})
    target_link_libraries(${test_name} ${PROJECT_NAME}_lib)
endfunction()

# nvblox ROS unit tests
add_nvblox_ros_unit_test(test_esdf_and_gradient_conversions)
add_nvblox_ros_unit_test(test_nvblox_node_params)
