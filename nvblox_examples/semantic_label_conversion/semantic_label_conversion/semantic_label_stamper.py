#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import json

import rclpy
from rclpy.node import Node
from nvblox_msgs.msg import SemanticLabelsStamped
from std_msgs.msg import String


class LabelsConverter(Node):
    def __init__(self) -> None:
        super().__init__('semantic_label_stamper')

        # Declare params
        self.declare_parameter('camera_1_enabled', True)
        self.declare_parameter('camera_2_enabled', True)
        self.declare_parameter('camera_1_name', 'left')
        self.declare_parameter('camera_2_name', 'right')

        # Get params
        camera_1_enabled = self.get_parameter(
            'camera_1_enabled').get_parameter_value().bool_value
        camera_2_enabled = self.get_parameter(
            'camera_2_enabled').get_parameter_value().bool_value
        camera_1_name = self.get_parameter(
            'camera_1_name').get_parameter_value().string_value
        camera_2_name = self.get_parameter(
            'camera_2_name').get_parameter_value().string_value

        if camera_1_enabled:
            self.init_camera(camera_1_name)
        if camera_2_enabled:
            self.init_camera(camera_2_name)

    def init_camera(self, camera_name: str) -> None:
        '''
        Initialize publishers and subscribers for a camera
        Args:
            camera_name (str): The name of the camera
        '''
        # Publisher
        labels_publisher = self.create_publisher(
            SemanticLabelsStamped, f"/semantic_conversion/{camera_name}/labels_stamped", 10)

        # Subscriber
        def on_camera_labels(msg): return self.on_labels(labels_publisher, msg)
        self.create_subscription(
            String, f"/{camera_name}/semantic_labels", on_camera_labels, 10)

    def on_labels(self, publisher, labels_string: String) -> None:
        '''
        Parse the string message for the labels and turn it into a custom message with Header

        Args:
            labels_string (String): String message coming out of IsaacSim
        '''
        # Load the string with json
        labels_data = json.loads(labels_string.data)
        # Get the timestamp issue
        time_data = labels_data['time_stamp']
        # Get the data without timestamp
        labels_string = labels_data.copy()
        labels_string.pop('time_stamp', None)
        # Build and publish message
        out_msg = SemanticLabelsStamped()
        out_msg.labels = json.dumps(labels_string)
        out_msg.header.stamp.sec = int(time_data['sec'])
        out_msg.header.stamp.nanosec = int(time_data['nanosec'])
        publisher.publish(out_msg)


def main():
    rclpy.init()
    semantic_label_stamper = LabelsConverter()
    rclpy.spin(semantic_label_stamper)
    semantic_label_stamper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
