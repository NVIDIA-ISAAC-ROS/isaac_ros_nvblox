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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap, Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('nvblox_examples_bringup')

    # Launch Arguments
    run_rviz_arg = DeclareLaunchArgument(
        'run_rviz', default_value='True',
        description='Whether to start RVIZ')
    run_nav2_arg = DeclareLaunchArgument(
        'run_nav2', default_value='True',
        description='Whether to run nav2')
    # NOTE(remos): When running Vslam make sure to set the global frame 
    #              to e.g. 'odom_vslam' to not clash with the Isaac Sim odometry frame
    run_vslam_arg = DeclareLaunchArgument(
        'run_vslam', default_value='False',
        description='Whether to run vslam')
    global_frame = LaunchConfiguration('global_frame',
                                       default='odom')

    # Create a shared container to hold composable nodes 
    # for speed ups through intra process communication.
    shared_container_name = "shared_nvblox_container"
    shared_container = Node(
        name=shared_container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen')

    # Nav2
    nav2_launch = GroupAction([
        SetRemap(src=['/nvblox_node/static_map_slice'],
                 dst=['/nvblox_human_node/static_map_slice']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bringup_dir, 'launch', 'nav2', 'nav2_isaac_sim.launch.py')),
            launch_arguments={'global_frame': global_frame}.items(),
            condition=IfCondition(LaunchConfiguration('run_nav2')))])

    # Vslam
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'perception', 'vslam.launch.py')]),
        launch_arguments={'output_odom_frame_name': global_frame,
                          'setup_for_isaac_sim': 'True', 
                          # Flatten VIO to 2D (assuming the robot only moves horizontally).
                          # This is needed to prevent vertical odometry drift.
                          'run_odometry_flattening': 'True',
                          'attach_to_shared_component_container': 'True',
                          'component_container_name': shared_container_name}.items(),
        condition=IfCondition(LaunchConfiguration('run_vslam')))

    # Semantic label conversion (convert Isaac Sim labels to mask image)
    semantic_label_conversion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('semantic_label_conversion'), 'launch',
            'semantic_label_conversion.launch.py')]))

    # Nvblox with humans
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'nvblox', 'nvblox_humans.launch.py')]),
        launch_arguments={'global_frame': global_frame,
                          'setup_for_isaac_sim': 'True', 
                          'attach_to_shared_component_container': 'True',
                          'component_container_name': shared_container_name}.items())

    # Rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'rviz', 'rviz.launch.py')]),
        launch_arguments={
            'config_name': 'isaac_sim_humans_example.rviz',
            'global_frame': global_frame}.items(),
        condition=IfCondition(LaunchConfiguration('run_rviz')))

    return LaunchDescription([
        run_rviz_arg,
        run_nav2_arg,
        run_vslam_arg,
        shared_container,
        nav2_launch,
        vslam_launch,
        semantic_label_conversion_launch,
        nvblox_launch,
        rviz_launch])
