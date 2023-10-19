# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import os
import pathlib
import subprocess
import time

from isaac_ros_test import IsaacROSBaseTest

import launch
from launch_ros.actions import Node

from nvblox_msgs.msg import DistanceMapSlice, Mesh

import pytest
import rclpy

_TEST_CASE_NAMESPACE = 'nvblox_test'


"""
    POL test for the Isaac ROS Nvblox node.

    1. Plays a ros bag recorded with Isaac SIM
    2. Nvblox node consumes the data and produces 3d Mesh
       and Map slice which is used as a costmap.
    3. This test expect the data to be available on topic nvblox_node/mesh
       and nvblox_node/static_map_slice.
"""


@pytest.mark.rostest
def generate_test_description():
    # Fetch git-lfs files
    print(subprocess.getoutput('git lfs pull -X ""'))

    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        namespace=IsaacROSNvBloxTest.generate_namespace(_TEST_CASE_NAMESPACE),
        parameters=[{'global_frame': 'odom'}, {'use_sim_time': True}],
        remappings=[('depth/camera_info', 'color/camera_info')],
        output='screen'
    )

    rosbag_play = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'play', os.path.dirname(__file__) +
             '/test_cases/rosbags/nvblox_pol',
             '--remap',
             'cmd_vel:=' +
             IsaacROSNvBloxTest.generate_namespace(_TEST_CASE_NAMESPACE) + '/cmd_vel',
             'clock:=' +
             IsaacROSNvBloxTest.generate_namespace(_TEST_CASE_NAMESPACE) + '/clock',
             'front/stereo_camera/left/depth:=' +
             IsaacROSNvBloxTest.generate_namespace(_TEST_CASE_NAMESPACE) + '/depth/image',
             'front/stereo_camera/left/camera_info:=' +
             IsaacROSNvBloxTest.generate_namespace(_TEST_CASE_NAMESPACE) + '/color/camera_info',
             'front/stereo_camera/left/rgb:=' +
             IsaacROSNvBloxTest.generate_namespace(_TEST_CASE_NAMESPACE) + '/color/image'],
        output='screen')

    return IsaacROSNvBloxTest.generate_test_description(
        [nvblox_node, rosbag_play])


class IsaacROSNvBloxTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))

    @ IsaacROSBaseTest.for_each_test_case('rosbags')
    def test_nvblox_node(self, test_folder):
        TIMEOUT = 10
        received_messages = {}
        self.generate_namespace_lookup(
            ['nvblox_node/mesh', 'nvblox_node/static_map_slice'], _TEST_CASE_NAMESPACE)
        subs = self.create_logging_subscribers(
            [('nvblox_node/mesh', Mesh), ('nvblox_node/static_map_slice', DistanceMapSlice)],
            received_messages,
            use_namespace_lookup=True, accept_multiple_messages=True)

        try:
            end_time = time.time() + TIMEOUT
            done = False

            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            if len(received_messages['nvblox_node/mesh']) > 0 and \
                    len(received_messages['nvblox_node/static_map_slice']) > 0:
                done = True

            self.assertTrue(
                done, 'Didnt recieve output on nvblox_node/mesh '
                      'or nvblox_node/static_map_slice topic')

        finally:
            [self.node.destroy_subscription(sub) for sub in subs]
