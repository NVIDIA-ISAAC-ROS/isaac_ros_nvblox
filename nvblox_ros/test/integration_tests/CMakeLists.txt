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

find_package(launch_testing_ament_cmake REQUIRED)

add_launch_test(isaac_ros_nvblox_pol.py)
add_launch_test(isaac_ros_nvblox_srv_save_and_load_map.py)
add_launch_test(isaac_ros_nvblox_srv_save_timings_and_rates.py)
add_launch_test(isaac_ros_nvblox_srv_save_ply.py)
