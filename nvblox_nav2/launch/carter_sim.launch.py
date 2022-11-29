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

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(
            get_package_share_directory(
                'nvblox_nav2'), 'params', 'carter_nav2.yaml'),
        description='Full path to param file to load')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation (Omniverse Isaac Sim) clock if true')

    nvblox_param_arg = DeclareLaunchArgument(
        'nvblox_params_file',
        default_value=os.path.join(
            get_package_share_directory('nvblox_nav2'), 'params', 'nvblox.yaml'
        )
    )
    use_depth_arg = DeclareLaunchArgument(
        'use_depth', default_value='True',
        description='Use depth as an input for nvblox reconstruction'
    )
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar', default_value='True',
        description='Use lidar as an input for nvblox reconstruction'
    )
    run_rviz_arg = DeclareLaunchArgument(
        'run_rviz', default_value='True',
        description='Whether to start RVIZ')
    run_nav2_arg = DeclareLaunchArgument(
        'run_nav2', default_value='True',
        description='Whether to run nav2')

    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(get_package_share_directory(
        'nvblox_nav2'), 'config', 'carter_nvblox_nav2.rviz')

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'rviz_launch.py')),
        launch_arguments={'namespace': '', 'use_namespace': 'False',
                          'autostart': 'True',
                          'rviz_config': rviz_config_dir}.items(),
        condition=IfCondition(LaunchConfiguration('run_rviz')))

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'navigation_launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'params_file': LaunchConfiguration('params_file'),
                          'autostart': 'True'}.items(),
        condition=IfCondition(LaunchConfiguration('run_nav2')))

    nvblox_node = Node(
        package='nvblox_ros', executable='nvblox_node',
        parameters=[LaunchConfiguration('nvblox_params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'use_depth': LaunchConfiguration('use_depth'),
                     'use_lidar': LaunchConfiguration('use_lidar')}],
        output='screen',
        remappings=[('depth/image', '/left/depth'),
                    ('depth/camera_info', '/left/camera_info'),
                    ('color/image', '/left/rgb'),
                    ('color/camera_info', '/left/camera_info'),
                    ('pointcloud', '/point_cloud')
                    ])

    map_to_odom_publisher = Node(
        package='tf2_ros', executable='static_transform_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        arguments=['0.0', '0.0', '-0.3', '0.0', '0.0', '0.0', 'map',
                   'odom'],
        condition=IfCondition(LaunchConfiguration('run_nav2')))

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        nvblox_param_arg,
        use_depth_arg,
        use_lidar_arg,
        run_rviz_arg,
        run_nav2_arg,
        rviz_launch,
        nav2_launch,
        nvblox_node,
        map_to_odom_publisher])
