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


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('run_realsense', True, cli=True)
    args.add_arg('run_rqt', True, cli=True)
    args.add_arg('output', 'None', cli=True)

    actions = args.get_launch_actions()

    # Launch realsense
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/sensors/realsense.launch.py',
            launch_arguments={
                'run_standalone': 'True',
            },
            condition=IfCondition(args.run_realsense)))

    # Rqt
    actions.append(
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            condition=IfCondition(args.run_rqt)))

    recording_started_msg =\
        '''\n\n\n
        -----------------------------------------------------
                    BAG RECORDING IS STARTING NOW

                 (make sure the realsense node is up)
        -----------------------------------------------------
        \n\n\n'''

    # Bag recording
    realsense_topics = [
        '/tf_static', '/camera/color/camera_info', '/camera/color/image_raw',
        '/camera/realsense_splitter_node/output/depth', '/camera/depth/camera_info',
        '/camera/realsense_splitter_node/output/infra_1', '/camera/infra1/camera_info',
        '/camera/realsense_splitter_node/output/infra_2', '/camera/infra2/camera_info'
    ]
    record_action = lu.record_rosbag(topics=" ".join(realsense_topics), bag_path=args.output)
    actions.append(
        TimerAction(period=10.0, actions=[record_action,
                                          lu.log_info(recording_started_msg)]))

    return LaunchDescription(actions)
