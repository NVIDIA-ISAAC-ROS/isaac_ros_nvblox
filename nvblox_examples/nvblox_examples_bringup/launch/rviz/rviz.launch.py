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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # Config
    config_name = LaunchConfiguration('config_name', default='default.rviz')
    config_path = PathJoinSubstitution([get_package_share_directory(
        'nvblox_examples_bringup'), 'config', 'rviz', config_name])
    global_frame = LaunchConfiguration('global_frame', default='odom')

    # Rviz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', config_path,       # set the config
                   '-f', global_frame],     # overwrite the global frame
        output='screen')

    return LaunchDescription([rviz])
