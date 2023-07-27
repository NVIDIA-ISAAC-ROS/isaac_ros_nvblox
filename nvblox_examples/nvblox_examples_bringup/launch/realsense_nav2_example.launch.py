# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 TINKER TWINS. All rights reserved.
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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    bringup_dir = get_package_share_directory('nvblox_examples_bringup')

    # Launch Arguments
    run_rviz_arg = DeclareLaunchArgument(
        'run_rviz', default_value='True',
        description='Whether to start RVIZ')
    run_nav2_arg = DeclareLaunchArgument(
        'run_nav2', default_value='True',
        description='Whether to run nav2')
    from_bag_arg = DeclareLaunchArgument(
        'from_bag', default_value='False',
        description='Whether to run from a bag or live realsense data')
    bag_path_arg = DeclareLaunchArgument(
        'bag_path', default_value='rosbag2*',
        description='Path of the bag (only used if from_bag == True)')
    global_frame = LaunchConfiguration('global_frame',
                                       default='odom')
    
    # Transform
    tf2_node = Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='base2camera_tf',
                    arguments = ['0.055', '0.000', '0.155',
                                '0.000', '1.571', '0.000',
                                'base_link', 'camera_link']
                ),

    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'nav2', 'nav2_realsense.launch.py')),
        condition=IfCondition(LaunchConfiguration('run_nav2')))
    
    # Realsense
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'sensors', 'realsense.launch.py')]),
        condition=UnlessCondition(LaunchConfiguration('from_bag')))

    # Vslam
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'perception', 'vslam.launch.py')]),
        launch_arguments={'output_odom_frame_name': global_frame}.items())

    # Nvblox
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'nvblox', 'nvblox.launch.py')]),
        launch_arguments={'setup_for_realsense': 'True',
                          'global_frame': global_frame
                          }.items())

    # Ros2 bag
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path')],
        shell=True, output='screen',
        condition=IfCondition(LaunchConfiguration('from_bag')))

    # Rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'rviz', 'rviz.launch.py')]),
        launch_arguments={'config_name': 'realsense_example.rviz',
                          'global_frame': global_frame}.items(),
        condition=IfCondition(LaunchConfiguration('run_rviz')))

    return LaunchDescription([
        run_rviz_arg,
        run_nav2_arg,
        from_bag_arg,
        bag_path_arg,
        # tf2_node,
        nav2_launch,
        realsense_launch,
        vslam_launch,
        nvblox_launch,
        bag_play,
        rviz_launch])
