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

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxCamera
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME

ZED_CAMERA_NAME = 'zed'


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('zed_camera_model')  # zed2 or zedx
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')
    actions = args.get_launch_actions()

    # Config file
    is_zedx = lu.is_equal(args.zed_camera_model, str(NvbloxCamera.zedx))
    zed2_config_file = lu.get_path('nvblox_examples_bringup', 'config/sensors/zed2.yaml')
    zedx_config_file = lu.get_path('nvblox_examples_bringup', 'config/sensors/zedx.yaml')
    config_file_camera = lu.if_else_substitution(is_zedx, str(zedx_config_file),
                                                 str(zed2_config_file))
    config_file_common = lu.get_path('nvblox_examples_bringup', 'config/sensors/zed_common.yaml')

    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = lu.get_path('zed_wrapper', 'urdf/zed_descr.urdf.xacro')

    # Robot State Publisher node (publishing static tfs for the camera)
    actions.append(
        Node(
            package='robot_state_publisher',
            namespace=ZED_CAMERA_NAME,
            executable='robot_state_publisher',
            name='zed_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro',
                    ' ',
                    str(xacro_path),
                    ' ',
                    'camera_name:=',
                    ZED_CAMERA_NAME,
                    ' ',
                    'camera_model:=',
                    args.zed_camera_model,
                ])
            }]))

    zed_node = ComposableNode(
        package='zed_components',
        namespace=ZED_CAMERA_NAME,
        name='zed_node',
        plugin='stereolabs::ZedCamera',
        parameters=[
            str(config_file_common),  # Common parameters
            config_file_camera,  # Camera related parameters
            {
                'general.camera_name': ZED_CAMERA_NAME
            },
        ])

    actions.append(
        lu.component_container(
            args.container_name, condition=IfCondition(lu.is_true(args.run_standalone))))
    actions.append(lu.load_composable_nodes(
        args.container_name,
        [zed_node],
    ))

    return LaunchDescription(actions)
