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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation (Omniverse Isaac Sim) clock if true')

    nvblox_param_dir_arg = DeclareLaunchArgument(
        'nvblox_params_file',
        default_value=os.path.join(
            get_package_share_directory('nvblox_examples_bringup'), 'config', 'nvblox', 'nvblox_base.yaml'
        ),
    )

    use_realsense_data_arg = DeclareLaunchArgument(
        'use_realsense_data', default_value='False',
        description='Use realsense data (otherwise use Omniverse Isaac Sim data)')

    # Carter Sim Remaps
    carter_sim_remaps = [('depth/image', '/front/stereo_camera/left/depth'),
                         ('depth/camera_info', '/front/stereo_camera/left/camera_info'),
                         ('color/image', '/front/stereo_camera/left/rgb'),
                         ('color/camera_info', '/front/stereo_camera/left/camera_info'),
                         ('pointcloud', '/point_cloud')]
    realsense_remaps = [('depth/image', '/camera/depth/image_rect_raw'),
                        ('depth/camera_info', '/camera/depth/camera_info'),
                        ('color/image', '/camera/color/image_raw'),
                        ('color/camera_info', '/camera/color/camera_info')]

    sim_recorder_node = Node(
        package='nvblox_performance_measurement',
        executable='results_collector_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        remappings=carter_sim_remaps,
        condition=UnlessCondition(LaunchConfiguration('use_realsense_data'))
    )

    sim_performance_node = Node(
        package='nvblox_performance_measurement',
        executable='nvblox_performance_measurement_node',
        parameters=[LaunchConfiguration('nvblox_params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        remappings=carter_sim_remaps,
        condition=UnlessCondition(LaunchConfiguration('use_realsense_data'))
    )

    realsense_recorder_node = Node(
        package='nvblox_performance_measurement',
        executable='results_collector_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        remappings=realsense_remaps,
        condition=IfCondition(LaunchConfiguration('use_realsense_data'))
    )

    realsense_performance_node = Node(
        package='nvblox_performance_measurement',
        executable='nvblox_performance_measurement_node',
        parameters=[LaunchConfiguration('nvblox_params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'depth_qos': 'SENSOR_DATA',
                     'color_qos': 'SENSOR_DATA'}],
        output='screen',
        remappings=realsense_remaps,
        condition=IfCondition(LaunchConfiguration('use_realsense_data'))
    )

    cpu_usage_node = Node(
        package='nvblox_cpu_gpu_tools', executable='cpu_percentage_node',
        parameters=[{
            'node_process_name':
            'nvblox_performance_measurement_node'}],
        output='screen')

    gpu_usage_node = Node(
        package='nvblox_cpu_gpu_tools', executable='gpu_percentage_node',
        parameters=[],
        output='screen')

    return LaunchDescription([use_sim_time_arg,
                              nvblox_param_dir_arg,
                              use_realsense_data_arg,
                              sim_recorder_node,
                              sim_performance_node,
                              realsense_recorder_node,
                              realsense_performance_node,
                              cpu_usage_node,
                              gpu_usage_node
                              ])
