# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from isaac_ros_launch_utils.all_types import LaunchDescription, SetParameter, IfCondition
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME

# NOTE: This launch file was implemented to run with the DOALS dataset,
#       which can be downloaded from https://projects.asl.ethz.ch/datasets/doku.php?id=doals
#       The ROS1 bag can be converted to ROS2 with rosbags:
#       https://docs.ros.org/en/noetic/api/ov_core/html/dev-ros1-to-ros2.html


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('rosbag',
                 'None',
                 description='Path to rosbag (running on sensor if not set).',
                 cli=True)
    args.add_arg('rosbag_args', '', description='Additional args for ros2 bag play.', cli=True)
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)
    args.add_arg('mode',
                 default=NvbloxMode.static,
                 choices=NvbloxMode.names(),
                 description='The nvblox mode.',
                 cli=True)
    args.add_arg('attach_to_container',
                 'False',
                 description='Add components to an existing component container.',
                 cli=True)
    args.add_arg('container_name',
                 NVBLOX_CONTAINER_NAME,
                 description='Name of the component container.')
    args.add_arg(
        'use_lidar_motion_compensation',
        '',
        description='Enable lidar motion compensation (empty string means use config default).',
        cli=True)
    actions = args.get_launch_actions()

    # Globally set use_sim_time if we're running from bag
    actions.append(SetParameter('use_sim_time', True))

    # Container
    actions.append(lu.component_container(NVBLOX_CONTAINER_NAME, log_level=args.log_level))

    # Nvblox
    actions.append(
        lu.include('nvblox_examples_bringup',
                   'launch/perception/nvblox.launch.py',
                   launch_arguments={
                       'container_name': args.container_name,
                       'mode': args.mode,
                       'camera': NvbloxCamera.os1,
                       'num_cameras': 0,
                       'lidar': 'True',
                       'use_lidar_motion_compensation': args.use_lidar_motion_compensation,
                   }))

    # Play ros2bag
    actions.append(
        lu.play_rosbag(bag_path=args.rosbag,
                       additional_bag_play_args=args.rosbag_args,
                       condition=IfCondition(lu.is_valid(args.rosbag))))

    # Visualization
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/visualization/rviz.launch.py',
            launch_arguments={
                'mode': args.mode,
                'camera': 'os1',
            },
        ))

    return LaunchDescription(actions)
