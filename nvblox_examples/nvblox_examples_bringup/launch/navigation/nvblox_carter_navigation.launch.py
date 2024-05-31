# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from typing import List

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def add_nvblox_carter_navigation(args: lu.ArgumentContainer) -> List[Action]:
    # Nav2 base parameter file
    actions = []
    nav_params_path = lu.get_path('nova_carter_navigation', 'params/nova_carter_navigation.yaml')
    actions.append(SetParametersFromFile(str(nav_params_path)))

    # Enabling nav2
    actions.append(
        lu.set_parameter(
            namespace='/local_costmap/local_costmap',
            parameter='plugins',
            value=['nvblox_layer', 'inflation_layer'],
        ))
    actions.append(
        lu.set_parameter(
            namespace='/global_costmap/global_costmap',
            parameter='plugins',
            value=['nvblox_layer', 'inflation_layer'],
        ))

    # Modifying nav2 parameters depending on nvblox mode
    mode = NvbloxMode[args.mode]
    if mode is NvbloxMode.static:
        costmap_topic_name = '/nvblox_node/static_map_slice'
    elif mode is NvbloxMode.dynamic:
        costmap_topic_name = '/nvblox_node/combined_map_slice'
    elif mode is NvbloxMode.people:
        costmap_topic_name = '/nvblox_human_node/combined_map_slice'
    else:
        raise Exception(f'Navigation in mode {mode} not implemented.')

    actions.append(
        lu.set_parameter(
            namespace='/global_costmap/global_costmap',
            parameter='nvblox_layer.nvblox_map_slice_topic',
            value=costmap_topic_name,
        ))
    # Increase global map to span full Isaac Sim scene (nvblox_sample_scene.usd).
    actions.append(
        lu.set_parameter(
            namespace='/global_costmap/global_costmap',
            parameter='width',
            value=60,
        ))
    actions.append(
        lu.set_parameter(
            namespace='/global_costmap/global_costmap',
            parameter='height',
            value=60,
        ))
    actions.append(
        lu.set_parameter(
            namespace='/global_costmap/global_costmap',
            parameter='origin_x',
            value=-30.0,
        ))
    actions.append(
        lu.set_parameter(
            namespace='/global_costmap/global_costmap',
            parameter='origin_y',
            value=-30.0,
        ))
    actions.append(
        lu.set_parameter(
            namespace='/local_costmap/local_costmap',
            parameter='nvblox_layer.nvblox_map_slice_topic',
            value=costmap_topic_name,
        ))

    # Running carter navigation
    actions.append(
        lu.include(
            'nova_carter_navigation',
            'launch/navigation.launch.py',
            launch_arguments={
                'navigation_container_name': args.container_name,
                'navigation_parameters_path': str(nav_params_path),
                'enable_mission_client': False
            },
        ))
    actions.append(lu.static_transform('map', 'odom'))

    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('mode')
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)

    args.add_opaque_function(add_nvblox_carter_navigation)
    return LaunchDescription(args.get_launch_actions())
