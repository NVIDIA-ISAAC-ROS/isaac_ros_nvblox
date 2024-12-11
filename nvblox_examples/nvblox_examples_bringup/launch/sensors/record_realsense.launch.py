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

from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def setup_record_bag(args: lu.ArgumentContainer) -> List[Action]:
    # Bag recording
    realsense_topics = [
        '/tf_static', '/camera0/imu', '/camera0/color/camera_info', '/camera0/color/image_raw',
        '/camera0/realsense_splitter_node/output/depth', '/camera0/depth/camera_info',
        '/camera0/realsense_splitter_node/output/infra_1', '/camera0/infra1/camera_info',
        '/camera0/realsense_splitter_node/output/infra_2', '/camera0/infra2/camera_info'
    ]
    camera_serial_numbers = str(args.camera_serial_numbers).split(',')
    for i in range(len(camera_serial_numbers)):
        name = 'camera' + str(i + 1)
        realsense_topics.append('/' + name + '/color/image_raw')
        realsense_topics.append('/' + name + '/color/camera_info')
        realsense_topics.append('/' + name + '/depth/image_rect_raw')
        realsense_topics.append('/' + name + '/depth/camera_info')

    record_action = lu.record_rosbag(topics=" ".join(realsense_topics), bag_path=args.output)

    recording_started_msg =\
        '''\n\n\n
        -----------------------------------------------------
                    BAG RECORDING IS STARTING NOW

                 (make sure the realsense node is up)
        -----------------------------------------------------
        \n\n\n'''

    # Note(xinjieyao: 2024/08/24): Recording based on multi-rs launch using RealSenseNodeFactory
    # could have siginficant frame drops, fot depth, color, infra, camera_info for topics published
    # by one camera, and for topics among all cameras.
    # Adding delay to as a temp fix.

    if args.run_realsense:
        delay_sec = len(camera_serial_numbers) * 10.0
    # Cameras are launched in another terminal with user operation delay added
    else:
        delay_sec = 10.0

    return [
        TimerAction(period=delay_sec, actions=[record_action, lu.log_info(recording_started_msg)])]


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('run_realsense', True, cli=True)
    args.add_arg('run_rqt', True, cli=True)
    args.add_arg('output', 'None', cli=True)
    args.add_arg('camera_serial_numbers', '',
                 description='List of the serial numbers of the cameras. (comma separated).'
                 'Optional in the case 1 camera, required in the case of multiple cameras.',
                 cli=True)
    args.add_arg('num_cameras', 1,
                 description='How many cameras to use.', cli=True)
    args.add_arg(
        'multicam_urdf_path',
        lu.get_path('nvblox_examples_bringup',
                    'config/urdf/4_realsense_carter_example_calibration.urdf.xacro'),
        description='Path to a URDF file describing the camera rig extrinsics. Only used in multicam.', cli=True)
    args.add_arg(
        'container_name',
        NVBLOX_CONTAINER_NAME,
        description='Name of the component container.')

    args.add_opaque_function(setup_record_bag)
    actions = args.get_launch_actions()

    # Launch realsense
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/sensors/realsense.launch.py',
            launch_arguments={
                'run_standalone': 'True',
                'camera_serial_numbers': args.camera_serial_numbers,
                'num_cameras': args.num_cameras,
                'container_name': args.container_name
            },
            condition=IfCondition(args.run_realsense)))

    # Single or Multi-realsense
    is_multi_cam = UnlessCondition(lu.is_equal(args.num_cameras, '1'))

    # TF transforms for multi-realsense
    actions.append(
        lu.add_robot_description(robot_calibration_path=args.multicam_urdf_path,
                                 condition=is_multi_cam)
    )

    # Rqt
    actions.append(
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            condition=IfCondition(args.run_rqt)))

    return LaunchDescription(actions)
