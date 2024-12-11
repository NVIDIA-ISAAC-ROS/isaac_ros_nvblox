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


def add_cameras(args: lu.ArgumentContainer) -> List[Action]:
    """Adds a realsense splitter for each camera name provided."""

    nodes = []
    camera_names = str(args.camera_names).split(',')
    assert len(camera_names) > 0
    for name in camera_names:
        nodes.append(ComposableNode(
            namespace=name,
            name='realsense_splitter_node',
            package='realsense_splitter',
            plugin='nvblox::RealsenseSplitterNode',
            parameters=[{
                'input_qos': 'SENSOR_DATA',
                'output_qos': 'SENSOR_DATA'
            }],
            remappings=[
                ('input/infra_1', f'/{name}/infra1/image_rect_raw'),
                ('input/infra_1_metadata', f'/{name}/infra1/metadata'),
                ('input/infra_2', f'/{name}/infra2/image_rect_raw'),
                ('input/infra_2_metadata', f'/{name}/infra2/metadata'),
                ('input/depth', f'/{name}/depth/image_rect_raw'),
                ('input/depth_metadata', f'/{name}/depth/metadata'),
                ('input/pointcloud', f'/{name}/depth/color/points'),
                ('input/pointcloud_metadata', f'/{name}/depth/metadata'),
            ]))
    return [lu.load_composable_nodes(args.container_name, nodes)]


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', 'realsense_splitter')
    args.add_arg('run_standalone', 'True')
    args.add_arg('camera_names', 'camera0')

    # Adding the cameras
    args.add_opaque_function(add_cameras)
    actions = args.get_launch_actions()
    actions.append(
        lu.component_container(
            args.container_name, condition=IfCondition(lu.is_true(args.run_standalone))))
    return LaunchDescription(actions)
