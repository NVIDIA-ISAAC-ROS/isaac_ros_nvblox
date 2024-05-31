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


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        'run_foxglove',
        False,
        description='Whether to run the foxglove bridge for visualization in foxglove.',
        cli=True)
    args.add_arg('run_rviz', True, description='Whether to run rviz2.', cli=True)
    args.add_arg('mode', NvbloxMode.static)
    args.add_arg('camera', NvbloxCamera.realsense)
    actions = args.get_launch_actions()

    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/visualization/foxglove_bridge.launch.py',
            condition=IfCondition(args.run_foxglove)))

    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/visualization/rviz.launch.py',
            launch_arguments={
                'mode': args.mode,
                'camera': args.camera,
            },
            condition=IfCondition(args.run_rviz)))

    return LaunchDescription(actions)
