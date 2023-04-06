#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
import math

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nvblox_performance_measurement_msgs.srv import FilePath as FilePathSrv
from nvblox_performance_measurement_msgs.srv import GetResults as GetResultsSrv
from rclpy.node import Node

import analyze_timestamps


class NvbloxRunner:
    def __init__(self):
        self.package_name = 'nvblox_performance_measurement'
        self.launchfile_name = 'carter_sim_benchmark.launch.py'
        self.arg_string = ''
        self.process = None

    def launch(self, realsense: bool = False):
        self.process = subprocess.Popen(
            f"exec ros2 launch {self.package_name} {self.launchfile_name} {self.arg_string} use_realsense_data:={realsense}",
            shell=True, preexec_fn=os.setsid)

    def kill(self):
        if self.process is not None:
            print('killing nvblox')
            self.process.kill()
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)


class BagLauncher:
    def __init__(self):
        self.process = None

    def launch(self, bag_path: str):
        self.process = subprocess.run(
            f"ros2 bag play {bag_path}", shell=True)

    def kill(self):
        if self.process is not None:
            print('killing bag')
            self.process.kill()


class ResultsGetter(Node):
    def __init__(self):
        super().__init__('results_getter_node')
        self.get_results_client = self.create_client(
            GetResultsSrv, '/results_collector_node/get_results')
        while not self.get_results_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def get_results(self):
        request = GetResultsSrv.Request()
        response = self.send_request_and_await_result(
            self.get_results_client, request)

        # Packing stamps into a dictionary
        stamps = {}
        for topic_stamps in response.topic_stamps:
            stamps[topic_stamps.topic_name] = np.array(topic_stamps.stamps)
        # Extracting the cpu/gpu usage samples into numpy array
        cpu_samples = np.array([msg.data
                                for msg in response.cpu_percentage_samples])
        gpu_samples = np.array([msg.data
                                for msg in response.gpu_percentage_samples])

        # Extract the timers string
        timers_string = response.timers_string.data

        return stamps, cpu_samples, gpu_samples, timers_string

    def send_request_and_await_result(self, client, request):
        result_future = self.send_request(client, request)
        return self.wait_for_result(result_future)

    def send_request(self, client, request):
        print('About to send request.')
        result_future = client.call_async(request)
        return result_future

    def wait_for_result(self, result_future):
        print('Picking up the results of the request')
        while rclpy.ok():
            rclpy.spin_once(self)
            if result_future.done():
                try:
                    response = result_future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    self.get_logger().info("Got response.")
                    return response
                break
            print('Didn\'t hear back yet...')


def save_results_files(stamps_dict: dict, cpu_samples: np.ndarray, gpu_samples: np.ndarray, timers_string: str, results_table: str, output_dir: Path, timestr: str = None):
    if timestr is None:
        timestr = time.strftime('%Y%m%d-%H%M%S')
    with open(output_dir / f"nvblox_timers_{timestr}.txt", 'w') as f:
        f.write(str(timers_string))
    with open(output_dir / f"benchmark_results_{timestr}.txt", 'w') as f:
        f.write(str(results_table))
    np.savetxt(output_dir / f"cpu_samples_{timestr}.txt", cpu_samples)
    np.savetxt(output_dir / f"gpu_samples_{timestr}.txt", gpu_samples)
    np.save(output_dir / f"message_stamps_{timestr}", stamps_dict)
    print(f"Wrote results in directory: {output_dir}")


def save_pulse_figure(stamps_dict: dict, filepath: Path):
    from plotly.subplots import make_subplots

    if len(stamps_dict['pointcloud']) > 0:
        pointclouds_present = True
    else:
        pointclouds_present = False

    if pointclouds_present:
        fig = make_subplots(rows=3, cols=1)
    else:
        fig = make_subplots(rows=2, cols=1)

    fig.add_trace(analyze_timestamps.get_pulses_plot(
        stamps_dict['/nvblox_node/depth_processed'], 2.0, line_name='depth processed'), row=1, col=1)
    fig.add_trace(analyze_timestamps.get_pulses_plot(
        stamps_dict['depth/image'], 1.0, line_name='depth released'), row=1, col=1)
    fig.add_trace(analyze_timestamps.get_pulses_plot(
        stamps_dict['/nvblox_node/color_processed'], 2.0, line_name='color processed'), row=2, col=1)
    fig.add_trace(analyze_timestamps.get_pulses_plot(
        stamps_dict['color/image'], 1.0, line_name='color released'), row=2, col=1)

    if pointclouds_present:
        fig.add_trace(analyze_timestamps.get_pulses_plot(
            stamps_dict['/nvblox_node/pointcloud_processed'], 2.0, line_name='pointcloud processed'), row=3, col=1)
        fig.add_trace(analyze_timestamps.get_pulses_plot(
            stamps_dict['pointcloud'], 1.0, line_name='pointcloud released'), row=3, col=1)

    fig.write_html(str(filepath))
    print(f'Wrote pulse figure to: {filepath}')


def remove_nan_values_from_dict(input: dict) -> dict:
    keys_to_delete = [key for key, value in input.items() if math.isnan(value)]
    for key in keys_to_delete:
        input.pop(key)
    return input


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(
        description="Run a benchmark of nvblox_ros.")
    parser.add_argument('bag_filepath', nargs='?',
                        help="Path to the bag to run the benchmark on.")
    parser.add_argument(
        "--realsense",
        default=False, action='store_const', const=True,
        help="Whether to use realsense settings.")
    parser.add_argument(
        "--output_dir",
        help="Directory in which to save the results files.")
    parser.add_argument(
        "--save_results_files", action='store_const', const=True,
        default=False,
        help="Flag indicating if we should save the results to files.")
    parser.add_argument(
        "--save_pulse_plot", action='store_const', const=True,
        default=False,
        help="Flag indicating if we should save the pulse plot.")
    parser.add_argument(
        "--save_kpi_json", action='store_const', const=True,
        default=False,
        help="Flag indicating if we should save the KPI table as a json.")

    args = parser.parse_args()

    # If no bag provided, use bags that come with the repo.
    # NOTE(alexmillane): Getting the dataset path here is kinda ugly, but I can't see a better way in ROS 2
    if args.bag_filepath is None:
        print("No bagfile provided. Using the bag distributed with this package (nvblox_performance_measurement).")
        nvblox_performance_measurement_share_dir = Path(
            get_package_share_directory('nvblox_performance_measurement'))
        workspace_dir = nvblox_performance_measurement_share_dir.parents[3]
        workspace_src_dir = workspace_dir / 'src'

        # Default bag locations (delivered via git lfs)
        if args.realsense:
            args.bag_filepath = workspace_src_dir / \
                Path('isaac_ros_nvblox/nvblox_performance_measurement/nvblox_performance_measurement/datasets/realsense_office')
        else:
            args.bag_filepath = workspace_src_dir / \
                Path('isaac_ros_nvblox/nvblox_ros/test/test_cases/rosbags/nvblox_pol')

    # Check the bag exists
    if not os.path.exists(args.bag_filepath):
        sys.exit(
            f"Bagfile at path: {args.bag_filepath} does not exist. \n\
                Either specify your own bag as an argument, or run \
                    git lfs pull to download the default bag.")

    print(f"Going to run performance test with bag: {args.bag_filepath}")

    # Run the test
    try:

        nvblox_startup_wait_time_s = 20.0
        print("Stating nvblox")
        nvblox_runner = NvbloxRunner()
        nvblox_runner.launch(args.realsense)
        print(
            f"Waiting for: {nvblox_startup_wait_time_s} for nvblox to start up before launching bag.")
        time.sleep(nvblox_startup_wait_time_s)

        print("Starting the bag")
        bag_launcher = BagLauncher()
        bag_launcher.launch(args.bag_filepath)
        print("Bag done")

        # Getting results (and printing them)
        results_getter = ResultsGetter()
        stamps_dict, cpu_samples, gpu_samples, timers_string = results_getter.get_results()

        print(
            f"\n\n{timers_string}\n\nBenchmarking Results\n-------------------------------------\n")
        results_table, results_table_dict = analyze_timestamps.make_results_table(
            stamps_dict, cpu_samples, gpu_samples)
        print(results_table)

        # Default file output location
        if args.output_dir is None:
            args.output_dir = '/tmp'
        output_dir = Path(args.output_dir)
        timestr = time.strftime('%Y%m%d-%H%M%S')
        print(f'Output directory is: {output_dir}')

        # Create the output dir if it doesn't exist already
        if not os.path.exists(output_dir):
            print(f'Output directory doesn\'t exist so creating it')
            os.makedirs(output_dir, exist_ok=True)

        # Saving the collected results
        if args.save_results_files:
            save_results_files(stamps_dict, cpu_samples, gpu_samples,
                               timers_string, results_table, output_dir, timestr=timestr)

        # Saving a plot
        if args.save_pulse_plot:
            save_pulse_figure(stamps_dict, output_dir / 'pulses.html')

        # Save results table as json
        if args.save_kpi_json:
            results_table_dict = remove_nan_values_from_dict(
                results_table_dict)
            timings_table_file = output_dir / 'ros_performance_kpis.json'
            print(f"Writing the timings table to: {timings_table_file}")
            with open(timings_table_file, "w") as timings_file:
                json.dump(results_table_dict, timings_file, indent=4)

    finally:
        # Kill
        nvblox_runner.kill()
        print("exiting")


if __name__ == '__main__':
    main()
