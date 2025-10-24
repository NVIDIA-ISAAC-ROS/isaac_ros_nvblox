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

import pytest

import launch_testing
from launch import LaunchDescription

from isaac_ros_test import IsaacROSBaseTest
import isaac_ros_launch_utils as lu
from nvblox_msgs.msg import DistanceMapSlice, Mesh

BAG_NAME = "galileo_static_3_2"
TIMEOUT = 120
BAG_PATH = os.path.join(lu.get_isaac_ros_ws_path(), 'isaac_ros_assets', 'rosbags', BAG_NAME)


@pytest.mark.rostest
def generate_test_description():
    # Launch perceptor configured for a single hawk.
    actions = []
    actions.append(
        lu.include('nvblox_examples_bringup',
                   'launch/realsense_example.launch.py',
                   launch_arguments={
                       'rosbag': BAG_PATH,
                       'num_cameras': 3,
                       'run_rviz': False,
                       'mode': 'dynamic',
                   }))
    # Required for ROS launch testing.
    actions.append(launch_testing.util.KeepAliveProc())
    actions.append(launch_testing.actions.ReadyToTest())
    return LaunchDescription(actions)


class IsaacROSNvBloxTest(IsaacROSBaseTest):
    filepath = pathlib.Path(__file__).parent

    def test_realsense_dynamics(self):
        received_messages = {}

        assert os.path.exists(BAG_PATH), "Test rosbag does not exist. Failing."

        subs = self.create_logging_subscribers(
            [('/nvblox_node/mesh', Mesh),
             ('/nvblox_node/combined_map_slice', DistanceMapSlice),
             ('/nvblox_node/dynamic_map_slice', DistanceMapSlice)
             ],
            received_messages,
            use_namespace_lookup=False,
            accept_multiple_messages=True)

        try:
            self.spin_node_until_messages_received(received_messages, TIMEOUT)
            self.assert_messages_received(received_messages)

        finally:
            [self.node.destroy_subscription(sub) for sub in subs]
