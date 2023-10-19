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

    bringup_dir = get_package_share_directory('nvblox_examples_bringup')
    base_config_dir = os.path.join(bringup_dir, 'config', 'nvblox')
    specialization_dir = os.path.join(base_config_dir, 'specializations')

    # Config files
    base_config = os.path.join(base_config_dir, 'nvblox_base.yaml')
    dynamics_config = os.path.join(specialization_dir, 'nvblox_dynamics.yaml')
    realsense_config = os.path.join(
        specialization_dir, 'nvblox_realsense.yaml')
    zed_config = os.path.join(
        specialization_dir, 'nvblox_zed.yaml')
    simulation_config = os.path.join(
        specialization_dir, 'nvblox_isaac_sim.yaml')

    # Conditionals for setup
    setup_for_dynamics = IfCondition(
        LaunchConfiguration('setup_for_dynamics', default='False'))
    setup_for_isaac_sim = IfCondition(
        LaunchConfiguration('setup_for_isaac_sim', default='False'))
    setup_for_realsense = IfCondition(
        LaunchConfiguration('setup_for_realsense', default='False'))
    setup_for_zed = IfCondition(
        LaunchConfiguration('setup_for_zed', default='False'))
    
    # Option to attach the nodes to a shared component container for speed ups through intra process communication.
    # Make sure to set the 'component_container_name' to the name of the component container you want to attach to.
    attach_to_shared_component_container_arg = LaunchConfiguration('attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration('component_container_name', default='nvblox_container')

    # If we do not attach to a shared component container we have to create our own container.
    nvblox_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=UnlessCondition(attach_to_shared_component_container_arg)
    )
    
    load_composable_nodes = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[
            ComposableNode(
            name='nvblox_node',
            package='nvblox_ros',
            plugin='nvblox::NvbloxNode')])

    group_action = GroupAction([

        # Set parameters with specializations
        SetParametersFromFile(base_config),
        SetParametersFromFile(dynamics_config,
                              condition=setup_for_dynamics),
        SetParametersFromFile(zed_config,
                              condition=setup_for_zed),
        SetParametersFromFile(realsense_config,
                              condition=setup_for_realsense),
        SetParametersFromFile(simulation_config,
                              condition=setup_for_isaac_sim),
        SetParameter(name='global_frame',
                     value=LaunchConfiguration('global_frame', default='odom')),

        # Remappings for realsense data
        SetRemap(src=['depth/image'],
                 dst=['/camera/realsense_splitter_node/output/depth'],
                 condition=setup_for_realsense),
        SetRemap(src=['depth/camera_info'],
                 dst=['/camera/depth/camera_info'],
                 condition=setup_for_realsense),
        SetRemap(src=['color/image'],
                 dst=['/camera/color/image_raw'],
                 condition=setup_for_realsense),
        SetRemap(src=['color/camera_info'],
                 dst=['/camera/color/camera_info'],
                 condition=setup_for_realsense),

        # Remappings for zed
	    SetRemap(src=['depth/image'],
                 dst=['/zed2/zed_node/depth/depth_registered'],
                 condition=setup_for_zed),
        SetRemap(src=['depth/camera_info'],
                 dst=['/zed2/zed_node/depth/camera_info'],
                 condition=setup_for_zed),
        SetRemap(src=['color/image'],
                 dst=['/zed2/zed_node/rgb/image_rect_color'],
                 condition=setup_for_zed),
        SetRemap(src=['color/camera_info'],
                 dst=['/zed2/zed_node/rgb/camera_info'],
                 condition=setup_for_zed),
        SetRemap(src=['pose'],
                 dst=['/zed2/zed_node/pose'],
                 condition=setup_for_zed),

        # Remappings for isaac sim data
        SetRemap(src=['depth/image'],
                 dst=['/front/stereo_camera/left/depth'],
                 condition=setup_for_isaac_sim),
        SetRemap(src=['depth/camera_info'],
                 dst=['/front/stereo_camera/left/camera_info'],
                 condition=setup_for_isaac_sim),
        SetRemap(src=['color/image'],
                 dst=['/front/stereo_camera/left/rgb'],
                 condition=setup_for_isaac_sim),
        SetRemap(src=['color/camera_info'],
                 dst=['/front/stereo_camera/left/camera_info'],
                 condition=setup_for_isaac_sim),
        SetRemap(src=['pointcloud'],
                 dst=['/point_cloud'],
                 condition=setup_for_isaac_sim),

        # Include the node container
        load_composable_nodes
    ])

    return LaunchDescription([nvblox_container, group_action])
