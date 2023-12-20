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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    send_buffer_limit = LaunchConfiguration('send_buffer_limit')
    send_buffer_limit_arg = DeclareLaunchArgument('send_buffer_limit',
                                                  default_value='10000000') # 10 mega bytes

    foxglove_bridge_node = Node(name='foxglove_bridge_node',
                                package='foxglove_bridge',
                                executable='foxglove_bridge',
                                parameters=[{
                                    'send_buffer_limit': send_buffer_limit,
                                    'max_qos_depth': 1
                                }])

    return LaunchDescription([send_buffer_limit_arg, foxglove_bridge_node])
