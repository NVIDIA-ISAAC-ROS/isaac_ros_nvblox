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

from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition


def generate_launch_description():
    # Check if ESS model is available
    # Steps to get the model can be found here:
    # https://nvidia-isaac-ros.github.io/robots/nova_carter.html#repository-setup
    engine_file_path = LaunchConfiguration(
        'engine_file_path',
        default="/workspaces/isaac_ros-dev/models/ess.engine")

    # NITROS nodes are required to start together to allow format negotiation
    # to take place before all the NITROS nodes are ready to start their underlying GXF graphs.
    # NOTE(remos): For big pipelines the negotiation time
    # can be increased with the type_negotiation_duration_s parameter.
    type_negotiation_duration_s_arg = LaunchConfiguration(
        'type_negotiation_duration_s', default=1)

    # Option to attach the nodes to a shared component container for speed ups through intra process communication.
    # Make sure to set the 'component_container_name' to the name of the component container you want to attach to.
    attach_to_shared_component_container_arg = LaunchConfiguration(
        'attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration(
        'component_container_name', default='ess_container')

    # If we do not attach to a shared component container we have to create our own container.
    ess_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=UnlessCondition(attach_to_shared_component_container_arg))

    load_composable_nodes = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[

            # ESS
            ComposableNode(
                name='ess_node',
                package='isaac_ros_ess',
                plugin='nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode',
                namespace='hawk',
                parameters=[{
                    'engine_file_path':
                    engine_file_path,
                    'threshold':
                    0.95,
                    'type_negotiation_duration_s':
                    type_negotiation_duration_s_arg,
                }],
                # NOTE(remos): We use the resized images here directly.
                # This way, the internal ESS resizing should be skipped.
                remappings=[('left/image_rect', 'left/image_resized'),
                            ('right/image_rect', 'right/image_resized'),
                            ('left/camera_info', 'left/camera_info_resized'),
                            ('right/camera_info', 'right/camera_info_resized')
                            ]),

            # Depth from disparity
            ComposableNode(
                name='disparity_to_depth',
                package='isaac_ros_stereo_image_proc',
                plugin=
                'nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode',
                namespace='hawk',
                parameters=[{
                    'type_negotiation_duration_s':
                    type_negotiation_duration_s_arg
                }]),
        ])

    return LaunchDescription([ess_container, load_composable_nodes])
