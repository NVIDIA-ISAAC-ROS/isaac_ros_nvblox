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


TOPIC_WHITELIST = [
    # nvblox
    '/nvblox_node/combined_esdf_pointcloud',
    '/nvblox_node/.*_layer',
    '/nvblox_node/esdf_slice_bounds',
    '/nvblox_node/mesh',
    '/nvblox_node/static_esdf_pointcloud',
    '/nvblox_node/human_voxels',
    # tf
    '/tf',
    '/tf_static'
    # CUVSLAM
    '/visual_slam/status',
    '/visual_slam/tracking/odometry',
    '/visual_slam/tracking/slam_path',
    '/visual_slam/tracking/vo_path',
    '/visual_slam/tracking/vo_pose',
    '/visual_slam/vis/landmarks_cloud',
    '/visual_slam/vis/observations_cloud',
]


def add_foxglove(args: lu.ArgumentContainer) -> List[Action]:

    params = [{
        'send_buffer_limit': int(args.send_buffer_limit),
        'max_qos_depth': 1,
        'use_compression': False,
        'capabilities': ['clientPublish', 'connectionGraph', 'assets'],
    }]

    if lu.is_true(args.use_foxglove_whitelist):
        params[0].update({'topic_whitelist': TOPIC_WHITELIST})

    actions = []
    actions.append(
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            parameters=params,
            # Use error log level to reduce terminal cluttering from "send_buffer_limit reached" warnings.
            arguments=['--ros-args', '--log-level', 'ERROR'],
        ))

    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('send_buffer_limit', 10000000)
    args.add_arg(
        'use_foxglove_whitelist',
        True,
        description='Disable visualization of bandwidth-heavy topics',
        cli=True)
    args.add_opaque_function(add_foxglove)

    return LaunchDescription(args.get_launch_actions())
