#!/usr/bin/env python3

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

import psutil

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class CpuPercentageNode(Node):

    def __init__(self):
        super().__init__('cpu_percentage_node')

        # Advertise CPU topic
        self.publisher_ = self.create_publisher(Float32, '~/cpu_percent', 10)

        # Declare params
        self.declare_parameter(
            'time_between_measurements_s', 1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='The time between sucessive measurements of the CPU load.'))
        self.declare_parameter(
            'measurement_interval_s', 0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='The interval passed to psutil.'))
        self.declare_parameter(
            'node_process_name', 'unset',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The name of the process to measure.'))

        # Get the initial values
        self._time_between_measurements_s = self.get_parameter(
            'time_between_measurements_s').get_parameter_value().double_value
        self._measurement_interval_s = self.get_parameter(
            'measurement_interval_s').get_parameter_value().double_value
        self._node_process_name = self.get_parameter(
            'node_process_name').get_parameter_value().string_value

        # We require the process name to be set
        if self._node_process_name == 'unset':
            self.get_logger().fatal("We require the parameter 'node_process_name' to be set.")
            self._ready = False
            return
        self.get_logger().info('Monitoring CPU usage for the process: ' + self._node_process_name)

        # Create a timer to measure cpu usage
        self.timer = self.create_timer(
            self._time_between_measurements_s, self.timer_callback)

        # Indicating that the setup was successful
        self._ready = True

        # We don't have a process to monitor yet
        self._process_id = -1

    def ready(self) -> bool:
        return self._ready

    def search_for_process(self) -> bool:
        self.get_logger().info('Searching for process: ' + self._node_process_name)
        pids = []
        for proc in psutil.process_iter(['pid', 'name']):
            if proc.name() == self._node_process_name:
                self.get_logger().info('Process found.')
                pids.append(proc.pid)
        if len(pids) == 1:
            self._process_id = pids[0]
            return True
        elif len(pids) == 0:
            self.get_logger().warning("Couldn't find the process.")
        else:
            self.get_logger().warning('Found more than one process. Not measuring.')
        return False

    def timer_callback(self):
        # Search for the process if required
        if self._process_id == -1:
            if self.search_for_process() is False:
                return

        # Check if the process has disappeared
        if not psutil.pid_exists(self._process_id):
            self.get_logger().warning('Process disappeared. Going back to search loop.')
            self._process_id = -1
            return

        # Get the CPU usage
        try:
            cpu_percent = psutil.Process(self._process_id).cpu_percent(
                interval=self._measurement_interval_s)
        except psutil.Error:
            # The process disappeared during measurement
            self.get_logger().warning('Process disappeared. Going back to search loop.')
            self._process_id = -1
            return

        # Publish the cpu usage
        msg = Float32()
        msg.data = cpu_percent
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    cpu_percentage_node = CpuPercentageNode()

    if cpu_percentage_node.ready():
        cpu_percentage_node.get_logger().info('Spinning.')
        rclpy.spin(cpu_percentage_node)

    # Shutdown
    cpu_percentage_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
