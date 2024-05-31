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

from launch_ros.actions import Node

from nvblox_msgs.srv import FilePath
from nvblox_ros_python_utils import nvblox_launch_test_utils

import pytest

_TEST_CASE_NAMESPACE = 'nvblox_test_srv_save_timings_and_rates'
"""
    Service test for the Isaac ROS Nvblox node.

    1. This test expect to validate save_timings service
    2. This test expect to validate save_rates service
"""


@pytest.mark.rostest
def generate_test_description():
    # Fetch git-lfs files
    print(subprocess.getoutput('git lfs pull -X ""'))

    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        namespace=IsaacROSNvBloxTest.generate_namespace(_TEST_CASE_NAMESPACE),
        parameters=[{
            'global_frame': 'odom'
        }],
        remappings=[('depth/camera_info', 'color/camera_info')],
        output='screen')

    return IsaacROSNvBloxTest.generate_test_description([nvblox_node])


class IsaacROSNvBloxTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))

    @ IsaacROSBaseTest.for_each_test_case('rosbags')
    def test_nvblox_node(self, test_folder):
        TIMEOUT = 10
        FPATH_TIMINGS = '/tmp/nvblox_timings.txt'
        FPATH_RATES = '/tmp/nvblox_rates.txt'

        # Create service client for save_timings service
        self.save_timings_cli = self.node.create_client(
            FilePath,
            '/isaac_ros_test/nvblox_test_srv_save_timings_and_rates/nvblox_node/save_timings')

        # Create service client for save_rates service
        self.save_rates_cli = self.node.create_client(
            FilePath,
            '/isaac_ros_test/nvblox_test_srv_save_timings_and_rates/nvblox_node/save_rates')

        time.sleep(TIMEOUT)

        nvblox_launch_test_utils.check_service_availability(
            self, self.save_timings_cli, 'save timings', TIMEOUT
        )
        self.save_timings_req = FilePath.Request()
        self.save_timings_req.file_path = FPATH_TIMINGS

        nvblox_launch_test_utils.check_service_availability(
            self, self.save_rates_cli, 'save rates', TIMEOUT
        )
        self.save_rates_req = FilePath.Request()
        self.save_rates_req.file_path = FPATH_RATES

        try:
            done = True

            save_timings_response = nvblox_launch_test_utils.get_service_response(
                self, self.save_timings_cli, self.save_timings_req, 'save timings', TIMEOUT
            )

            save_rates_response = nvblox_launch_test_utils.get_service_response(
                self, self.save_rates_cli, self.save_rates_req, 'save rates', TIMEOUT
            )

            done = (
                done and
                nvblox_launch_test_utils.is_service_succeeded(
                    self, save_timings_response, 'save timings', FPATH_TIMINGS
                ) and
                nvblox_launch_test_utils.is_service_succeeded(
                    self, save_rates_response, 'save rates', FPATH_RATES
                )
            )

            self.assertTrue(
                done, 'Save Timings or Save Rates service did not run successfully')

        finally:
            if os.path.exists(FPATH_TIMINGS):
                os.remove(FPATH_TIMINGS)
            if os.path.exists(FPATH_RATES):
                os.remove(FPATH_RATES)
