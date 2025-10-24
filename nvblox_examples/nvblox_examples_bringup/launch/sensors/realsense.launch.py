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

from typing import List, Optional

from isaac_ros_launch_utils.all_types import (
    Action, ComposableNode, LaunchDescription, TimerAction, IfCondition)
import isaac_ros_launch_utils as lu
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME

EMITTER_FLASHING_CONFIG_FILE_PATH = lu.get_path(
    'nvblox_examples_bringup',
    'config/sensors/realsense_emitter_flashing.yaml')
EMITTER_ON_CONFIG_FILE_PATH = lu.get_path(
    'nvblox_examples_bringup',
    'config/sensors/realsense_emitter_on.yaml')

# By default our behaviour is:
# - Run the splitter on camera0,
# - Don't run the splitter on the remaining cameras.
# NOTE(alexmillane, 16.08.2024): At the moment this is the *only* behaviour we support.


def get_default_run_splitter_list(num_cameras: int) -> List[bool]:
    run_splitter_list = [False] * num_cameras
    run_splitter_list[0] = True
    return run_splitter_list


def get_camera_node(
        camera_name: str,
        config_file_path: str,
        serial_number: Optional[int] = None) -> ComposableNode:
    parameters = []
    parameters.append(config_file_path)
    parameters.append({'camera_name': camera_name})
    if serial_number:
        parameters.append({'serial_no': str(serial_number)})
    realsense_node = ComposableNode(
        name=camera_name,
        namespace='',
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=parameters)
    return realsense_node


def get_splitter_node(camera_name: str) -> ComposableNode:
    realsense_splitter_node = ComposableNode(
        namespace=camera_name,
        name='realsense_splitter_node',
        package='realsense_splitter',
        plugin='nvblox::RealsenseSplitterNode',
        parameters=[{
            'input_qos': 'SENSOR_DATA',
            'output_qos': 'SENSOR_DATA'
        }],
        remappings=[
            ('input/infra_1', f'/{camera_name}/infra1/image_rect_raw'),
            ('input/infra_1_metadata', f'/{camera_name}/infra1/metadata'),
            ('input/infra_2', f'/{camera_name}/infra2/image_rect_raw'),
            ('input/infra_2_metadata', f'/{camera_name}/infra2/metadata'),
            ('input/depth', f'/{camera_name}/depth/image_rect_raw'),
            ('input/depth_metadata', f'/{camera_name}/depth/metadata'),
            ('input/pointcloud', f'/{camera_name}/depth/color/points'),
            ('input/pointcloud_metadata', f'/{camera_name}/depth/metadata'),
        ])
    return realsense_splitter_node


def add_cameras(args: lu.ArgumentContainer) -> List[Action]:
    """Adds a camera and (optional) realsense splitter for each camera up to num_cameras."""

    # Serial numbers.
    if args.camera_serial_numbers == '':
        camera_serial_numbers = [None]
    else:
        camera_serial_numbers = str(args.camera_serial_numbers).split(',')
    assert len(camera_serial_numbers) > 0
    # Run splitter list. I.e. a list of bools indicating per-camera if we should run a splitter.
    run_splitter_list = get_default_run_splitter_list(len(camera_serial_numbers))
    assert len(camera_serial_numbers) == len(run_splitter_list)
    # Number of cameras to run
    num_cameras = int(args.num_cameras)
    assert num_cameras <= len(camera_serial_numbers)

    actions = []
    for idx in range(num_cameras):
        camera_serial_number = camera_serial_numbers[idx]
        run_splitter = run_splitter_list[idx]
        nodes = []
        camera_name = f'camera{idx}'
        # Config file
        if run_splitter:
            config_file_path = EMITTER_FLASHING_CONFIG_FILE_PATH
        else:
            config_file_path = EMITTER_ON_CONFIG_FILE_PATH
        # Realsense
        log_message = lu.log_info(
            f'Starting realsense with name: {camera_name}, running splitter: {run_splitter}')
        nodes.append(
            get_camera_node(
                camera_name=camera_name,
                config_file_path=config_file_path,
                serial_number=camera_serial_number,
            ))
        # Splitter
        if run_splitter:
            nodes.append(
                get_splitter_node(
                    camera_name=camera_name,
                ))
        # Note(xinjieyao: 2024/08/24): Multi-rs launch use RealSenseNodeFactory could be unstable
        # Camera node may fail to launch without any ERROR or app crashes
        # Adding delay for cameras after the first camera bringup (including splitter) as temp fix
        actions.append(
            TimerAction(
                period=idx * 10.0, actions=[lu.load_composable_nodes(args.container_name, nodes)]))
        actions.append(log_message)

    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')
    args.add_arg('camera_serial_numbers', '')
    args.add_arg('num_cameras', 1)

    # Adding the cameras
    args.add_opaque_function(add_cameras)
    actions = args.get_launch_actions()
    actions.append(
        lu.component_container(
            args.container_name, condition=IfCondition(lu.is_true(args.run_standalone))))
    return LaunchDescription(actions)
