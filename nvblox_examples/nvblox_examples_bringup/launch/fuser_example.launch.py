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

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxCamera, NvbloxMode


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('path', 'None', description='Path to dataset', cli=True)
    args.add_arg(
        'type',
        '3dmatch',
        choices=['3dmatch', 'redwood', 'replica', 'rosbag'],
        description='Type of dataset',
        cli=True)
    args.add_arg(
        'update_on_key',
        'False',
        description='Whether to fuse the dataset full speed or update on key inputs from terminal.',
        cli=True)
    args.add_arg(
        'num_frames',
        -1,
        description='Number of frames to process. Negative means process all.',
        cli=True)
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)
    actions = args.get_launch_actions()

    # Load the yaml containing the config.
    fuser_config = lu.get_path('nvblox_examples_bringup', 'config/nvblox/fuser.yaml')

    # Fuser node
    actions.append(
        Node(
            name='fuser_node',
            executable='fuser_node',
            package='nvblox_ros',
            parameters=[
                fuser_config, {
                    'dataset_type': args.type,
                    'dataset_path': args.path,
                    'update_on_key': args.update_on_key,
                    'number_of_frames_to_integrate': args.num_frames,
                }
            ]))

    # Visualization
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/visualization/visualization.launch.py',
            launch_arguments={
                'mode': NvbloxMode.static,
                'camera': NvbloxCamera.fuser
            }))

    return LaunchDescription(actions)
