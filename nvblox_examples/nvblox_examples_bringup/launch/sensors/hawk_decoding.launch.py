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
from launch.actions import GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import LoadComposableNodes, Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (SetParameter, SetParametersFromFile, SetRemap)
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
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
        'hawk_decoding_container', default='hawk_processing_container')

    # If we do not attach to a shared component container we have to create our own container.
    hawk_bag_decoding_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=UnlessCondition(attach_to_shared_component_container_arg))

    load_composable_nodes = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[

            # Left decoder node
            ComposableNode(
                name='left_decoder_node',
                package='isaac_ros_h264_decoder',
                plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
                namespace='hawk',
                parameters=[{
                    'type_negotiation_duration_s':
                    type_negotiation_duration_s_arg,
                }],
                remappings=[
                    ('image_compressed', 'left/image_compressed'),
                    ('image_uncompressed', 'left/image_raw'
                     )  # This is the input to hawk_processing
                ]),

            # Right decoder node
            ComposableNode(
                name='right_decoder_node',
                package='isaac_ros_h264_decoder',
                plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
                namespace='hawk',
                parameters=[{
                    'type_negotiation_duration_s':
                    type_negotiation_duration_s_arg,
                }],
                remappings=[
                    ('image_compressed', 'right/image_compressed'),
                    ('image_uncompressed', 'right/image_raw'
                     )  # This is the input to hawk_processing
                ]),
        ])

    return LaunchDescription(
        [hawk_bag_decoding_container, load_composable_nodes])
