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

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

try:
    import gpustat
    on_jetson_flag = False
except ImportError:
    from jtop import jtop
    on_jetson_flag = True


class GpuPercentageNode(Node):

    def __init__(self, on_jetson_flag: bool):
        super().__init__('gpu_percentage_node')

        self._on_jetson_flag = on_jetson_flag
        if self._on_jetson_flag:
            self.get_logger().info("Detected that we're on jetson.")
        else:
            self.get_logger().info("Detected that we're on x86.")

        # Advertise CPU topic
        self.publisher_ = self.create_publisher(Float32, '~/gpu_percent', 10)

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
            'gpu_index', 0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='The index of the GPU to track on a multi GPU system.'))

        # Get the initial values
        self._time_between_measurements_s = self.get_parameter(
            'time_between_measurements_s').get_parameter_value().double_value
        self._measurement_interval_s = self.get_parameter(
            'measurement_interval_s').get_parameter_value().double_value

        # Index of the GPU to monitor
        self._gpu_index = 0
        # If we have multiple GPUs, read the index to track from ROS
        if self.get_number_of_gpus() != 1:
            self.get_logger().info('Multi GPU system detected.')
            self._gpu_index = self.get_parameter(
                'gpu_index').get_parameter_value().integer_value
            if self._gpu_index >= self.get_number_of_gpus() or self._gpu_index < 0:
                self.get_logger().fatal(
                    f"""Detected {self.get_number_of_gpus()} GPUs. Requested to track
                        GPU index {self._gpu_index}. Not possible. Failing.""")
                self._ready = False
                return
            self.get_logger().info(
                f'Tracking GPU usage for GPU index: {self._gpu_index}.')

        # Indicating that the setup was successful
        self._ready = True

        # Start up jtop if we're on Jetson
        if self._on_jetson_flag:
            self._jetson = jtop()
            self._jetson.start()

        # Create a timer to measure cpu usage
        self.timer = self.create_timer(
            self._time_between_measurements_s, self.timer_callback)

    def ready(self) -> bool:
        return self._ready

    def get_number_of_gpus(self):
        if self._on_jetson_flag:
            return 1
        else:
            return len(gpustat.new_query().gpus)

    def timer_callback(self):

        # Get the GPU usage
        if self._on_jetson_flag:
            gpu_percent = float(self._jetson.stats['GPU'])
        else:
            gpu_percent = float(gpustat.new_query().gpus[0].utilization)

        # Publish the cpu usage
        msg = Float32()
        msg.data = gpu_percent
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    gpu_percentage_node = GpuPercentageNode(on_jetson_flag)

    if gpu_percentage_node.ready():
        gpu_percentage_node.get_logger().info('Spinning.')
        rclpy.spin(gpu_percentage_node)

    # Shutdown
    gpu_percentage_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
