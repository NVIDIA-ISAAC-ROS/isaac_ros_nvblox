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

import os
import pathlib
import subprocess
import time

from isaac_ros_test import IsaacROSBaseTest

import launch
from launch_ros.actions import Node

from nvblox_msgs.srv import FilePath
from nvblox_ros_python_utils import nvblox_launch_test_utils

import pytest

_TEST_CASE_NAMESPACE = 'nvblox_test_srv_save_ply'
"""
    Service test for the Isaac ROS Nvblox node.

    1. Plays a ros bag recorded with Isaac SIM
    2. Nvblox node consumes the data and produces 3d Mesh
       and Map slice which is used as a costmap.
    3. This test expect to validate save_ply service
"""


@pytest.mark.rostest
def generate_test_description():
    # Fetch git-lfs files
    print(subprocess.getoutput('git lfs pull -X ""'))

    test_namespace = IsaacROSNvBloxTest.generate_namespace(_TEST_CASE_NAMESPACE)

    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        namespace=test_namespace,
        parameters=[{
            'global_frame': 'odom'
        }],
        remappings=[('depth/camera_info', 'color/camera_info')],
        output='screen')

    rosbag_play = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', '-l',
            os.path.dirname(__file__) + '/test_cases/rosbags/nvblox_pol', '--remap',
            'cmd_vel:=' + test_namespace + '/cmd_vel', 'clock:=' + test_namespace + '/clock',
            'front_stereo_camera/depth/ground_truth:=' + test_namespace + '/camera_0/depth/image',
            'front_stereo_camera/depth/camera_info:=' + test_namespace +
            '/camera_0/depth/camera_info', 'front_stereo_camera/left/camera_info:=' +
            test_namespace + '/camera_0/color/camera_info',
            'front_stereo_camera/left/image_raw:=' + test_namespace + '/camera_0/color/image'
        ],
        output='screen')

    return IsaacROSNvBloxTest.generate_test_description([nvblox_node, rosbag_play])


class IsaacROSNvBloxTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))

    @ IsaacROSBaseTest.for_each_test_case('rosbags')
    def test_nvblox_node(self, test_folder):
        TIMEOUT = 15
        FPATH_PLY = '/tmp/super_cool_map.ply'

        # Create service client for save_ply service
        self.save_ply_cli = self.node.create_client(
            FilePath, '/isaac_ros_test/nvblox_test_srv_save_ply/nvblox_node/save_ply')

        time.sleep(TIMEOUT)

        nvblox_launch_test_utils.check_service_availability(
            self, self.save_ply_cli, 'save ply', TIMEOUT
        )
        self.save_ply_req = FilePath.Request()
        self.save_ply_req.file_path = FPATH_PLY

        try:
            done = True

            save_ply_response = nvblox_launch_test_utils.get_service_response(
                self, self.save_ply_cli, self.save_ply_req, 'save ply', TIMEOUT
            )

            done = (
                done and
                nvblox_launch_test_utils.is_service_succeeded(
                    self, save_ply_response, 'save ply', FPATH_PLY
                )
            )

            self.assertTrue(
                done, 'Save ply service did not run successfully')

        finally:
            if os.path.exists(FPATH_PLY):
                os.remove(FPATH_PLY)
