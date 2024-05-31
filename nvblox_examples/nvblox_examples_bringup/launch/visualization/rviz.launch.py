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

import pathlib

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera


def add_rviz(args: lu.ArgumentContainer) -> list[Action]:
    if lu.is_valid(args.rviz_config):
        rviz_config_path = pathlib.Path(args.rviz_config)
    else:
        mode = NvbloxMode[args.mode]
        camera = NvbloxCamera[args.camera]

        if mode is NvbloxMode.people:
            rviz_config_name = str(camera) + "_people_example.rviz"
        elif mode is NvbloxMode.dynamic:
            rviz_config_name = str(camera) + "_dynamics_example.rviz"
        else:
            rviz_config_name = str(camera) + "_example.rviz"

        rviz_config_path = lu.get_path('nvblox_examples_bringup',
                                       'config/visualization/' + rviz_config_name)

    actions = []
    assert rviz_config_path.exists(), f'Rviz config {rviz_config_path} does not exist.'
    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_config_path)],
            output="screen"))
    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        'rviz_config',
        'None',
        description='Path to rviz config (using example config if not set).',
        cli=True)
    args.add_arg('mode', NvbloxMode.static)
    args.add_arg('camera', NvbloxCamera.realsense)

    args.add_opaque_function(add_rviz)
    return LaunchDescription(args.get_launch_actions())
