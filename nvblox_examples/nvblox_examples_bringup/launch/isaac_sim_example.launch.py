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

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        'rosbag',
        'None',
        description='Path to rosbag (running with Isaac Sim if not set).',
        cli=True)
    args.add_arg('rosbag_args', '', description='Additional args for ros2 bag play.', cli=True)
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)
    args.add_arg(
        'mode',
        NvbloxMode.static,
        choices=NvbloxMode.names(),
        description='The nvblox mode.',
        cli=True)
    args.add_arg(
        'num_cameras',
        3,
        choices=['0', '1', '3'],
        description='Number of cameras that should be used for 3d reconstruction',
        cli=True)
    args.add_arg(
        'lidar', False, description='Whether to use 3d lidar for 3d reconstruction', cli=True)
    args.add_arg(
        'navigation',
        True,
        description='Whether to enable nav2 for navigation in Isaac Sim.',
        cli=True)
    actions = args.get_launch_actions()

    # Globally set use_sim_time
    actions.append(SetParameter('use_sim_time', True))

    # Navigation
    # NOTE: needs to be called before the component container because it modifies params globally
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/navigation/nvblox_carter_navigation.launch.py',
            launch_arguments={
                'container_name': NVBLOX_CONTAINER_NAME,
                'mode': args.mode,
            },
            condition=IfCondition(lu.is_true(args.navigation))))

    # Container
    actions.append(
        lu.component_container(
            NVBLOX_CONTAINER_NAME, container_type='isolated', log_level=args.log_level))

    # People segmentation
    actions.append(
        lu.include(
            'semantic_label_conversion',
            'launch/semantic_label_conversion.launch.py',
            condition=IfCondition(lu.has_substring(args.mode, NvbloxMode.people_segmentation))))

    # Nvblox
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/perception/nvblox.launch.py',
            launch_arguments={
                'container_name': NVBLOX_CONTAINER_NAME,
                'mode': args.mode,
                'camera': NvbloxCamera.isaac_sim,
                'num_cameras': args.num_cameras,
                'lidar': args.lidar,
            }))

    # Play ros2bag
    actions.append(
        lu.play_rosbag(
            bag_path=args.rosbag,
            additional_bag_play_args=args.rosbag_args,
            condition=IfCondition(lu.is_valid(args.rosbag))))

    # Visualization
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/visualization/visualization.launch.py',
            launch_arguments={
                'mode': args.mode,
                'camera': NvbloxCamera.isaac_sim,
            }))

    return LaunchDescription(actions)
