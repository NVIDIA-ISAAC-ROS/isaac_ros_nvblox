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
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import UnlessCondition


def generate_launch_description():

    # Config file
    config_file_camera = os.path.join(
        get_package_share_directory('nvblox_examples_bringup'),
        'config', 'sensors', 'zed2.yaml')

    config_file_common = os.path.join(
        get_package_share_directory('nvblox_examples_bringup'),
        'config', 'sensors', 'zed_common.yaml')
    
    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf',
        'zed_descr.urdf.xacro'
    )

    # Option to attach the nodes to a shared component container for speed ups through intra process communication.
    # Make sure to set the 'component_container_name' to the name of the component container you want to attach to.
    attach_to_shared_component_container_arg = LaunchConfiguration(
        'attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration(
        'component_container_name', default='realsense_container')

    # If we do not attach to a shared component container we have to create our own container.
    zed2_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=UnlessCondition(attach_to_shared_component_container_arg)
    )

    # Robot State Publisher node (publishing static tfs for the camera)
    rsp_node = Node(
        package='robot_state_publisher',
        namespace='zed2',
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', 'zed2', ' ',
                    'camera_model:=', 'zed2', ' ',
                    'base_frame:=', 'base_link', ' ',
                ])
        }]
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[
            # Zed2 wrapper node
            ComposableNode(
                package='zed_components',
                namespace='zed2',
                name='zed_node',
                plugin='stereolabs::ZedCamera',
                parameters=[
                    # YAML files
                    config_file_common,  # Common parameters
                    config_file_camera,  # Camera related parameters
                    ]
            ),
            ])

    return LaunchDescription([
        rsp_node, 
        zed2_container, 
        load_composable_nodes
        ])
