#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import analyze_timestamps
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nvblox_performance_measurement_msgs.srv import GetResults as GetResultsSrv
from rclpy.node import Node


class NvbloxRunner:
    def __init__(self, realsense, with_humans, with_dynamics, model_name, model_repository_paths, input_binding_names, output_binding_names):
        self.package_name = 'nvblox_performance_measurement'
        self.arg_string = ''
        self.process = None
        self.launchfile_name = 'isaac_sim_benchmark.launch.py'
        if realsense and with_humans:
            self.launchfile_name = 'realsense_humans_benchmark.launch.py'
        elif realsense:
            self.launchfile_name = 'realsense_benchmark.launch.py'
        elif with_humans:
            self.launchfile_name = 'isaac_sim_humans_benchmark.launch.py'

        if with_dynamics:
            self.arg_string += 'mapping_type:=dynamic' + ' '
        elif with_humans:
            self.arg_string += 'model_name:=' + model_name + ' '
            self.arg_string += 'model_repository_paths:=' + model_repository_paths + ' '
            self.arg_string += 'input_binding_names:=' + str(input_binding_names) + ' '
            self.arg_string += 'output_binding_names:=' + str(output_binding_names)


    def launch(self):
        self.process = subprocess.Popen(
            f"exec ros2 launch {self.package_name} {self.launchfile_name} {self.arg_string}",
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

        # Extract network performance measurement
        network_mean_iou_samples = np.array([msg.data
                                             for msg in response.network_mean_iou_percentage_samples])

        # Extract the timers string
        timers_string = response.timers_string.data

        return stamps, cpu_samples, gpu_samples, network_mean_iou_samples, timers_string

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


def save_results_files(stamps_dict: dict, cpu_samples: np.ndarray,
                       gpu_samples: np.ndarray, network_mean_iou_samples: np.ndarray, timers_string: str,
                       results_table: str, output_dir: Path, timestr: str = None):
    if timestr is None:
        timestr = time.strftime('%Y%m%d-%H%M%S')
    with open(output_dir / f"nvblox_timers_{timestr}.txt", 'w') as f:
        f.write(str(timers_string))
    with open(output_dir / f"benchmark_results_{timestr}.txt", 'w') as f:
        f.write(str(results_table))
    np.savetxt(
        output_dir / f"cpu_samples_{timestr}.txt", cpu_samples)
    np.savetxt(output_dir / f"gpu_samples_{timestr}.txt", gpu_samples)
    np.savetxt(
        output_dir / f"network_mean_iou_samples_{timestr}.txt", network_mean_iou_samples)
    np.save(output_dir / f"message_stamps_{timestr}", stamps_dict)
    print(f"Wrote results in directory: {output_dir}")


def save_pulse_figure(stamps_dict: dict, filepath: Path, with_humans: bool = False):
    from plotly.subplots import make_subplots

    if len(stamps_dict['pointcloud']) > 0:
        pointclouds_present = True
    else:
        pointclouds_present = False

    if pointclouds_present:
        fig = make_subplots(rows=3, cols=1)
    else:
        fig = make_subplots(rows=2, cols=1)

    fig.add_trace(
        analyze_timestamps.get_pulses_plot(
            stamps_dict['/nvblox_human_node/depth_processed'],
            2.0, line_name='depth processed'),
        row=1, col=1)

    fig.add_trace(
        analyze_timestamps.get_pulses_plot(
            stamps_dict['/nvblox_human_node/color_processed'],
            2.0, line_name='color processed'),
        row=2, col=1)

    if pointclouds_present:
        fig.add_trace(
            analyze_timestamps.get_pulses_plot(
                stamps_dict['/nvblox_human_node/pointcloud_processed'],
                2.0, line_name='pointcloud processed'),
            row=3, col=1)

    fig.add_trace(analyze_timestamps.get_pulses_plot(
        stamps_dict['depth/image'], 1.0, line_name='depth released'), row=1, col=1)

    fig.add_trace(analyze_timestamps.get_pulses_plot(
        stamps_dict['color/image'], 1.0, line_name='color released'), row=2, col=1)

    if pointclouds_present:
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
        action='store_true',
        help="Whether to use realsense settings.")
    parser.add_argument(
        "--with_humans",
        action='store_true',
        help="Whether to run human detection.")
    parser.add_argument(
        "--with_dynamics",
        action='store_true',
        help="Whether to run dynamic detection.")
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
    parser.add_argument(
        "--save_kpis_in_namespace",
        help="Namespace to save the kpis")
    parser.add_argument(
        "--model_name",
        default='peoplesemsegnet',
        help="Path to the network model file")
    parser.add_argument(
        "--model_repository_paths",
        default="['/workspaces/isaac_ros-dev/models']",
        help="Path to the tensorrt model file")
    parser.add_argument(
        "--input_binding_name",
        default=['input_1:0'],
        help="Input binding name for the tensorrt model")
    parser.add_argument(
        "--output_binding_name",
        default=['argmax_1'],
        help="Output binding name for the tensorrt model")

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
            args.bag_filepath = workspace_src_dir / Path(
                'isaac_ros_nvblox/nvblox_performance_measurement/nvblox_performance_measurement/datasets/realsense_office')
        else:
            args.bag_filepath = workspace_src_dir / \
                Path('isaac_ros_nvblox/nvblox_ros/test/test_cases/rosbags/nvblox_pol')

    # Check the bag exists
    if not os.path.exists(args.bag_filepath):
        sys.exit(
            f"Bagfile at path: {args.bag_filepath} does not exist. \n\
                Either specify your own bag as an argument, or run \
                    git lfs pull to download the default bag.")

    print("")
    print(f"Going to run performance test with bag: {args.bag_filepath}")
    if args.realsense: 
        print("Running realsense benchmark.")
    else:
        print("Running Isaac Sim benchmark.")

    if args.with_humans:
        print("Human detection activated.")
    elif args.with_dynamics:
        print("Dynamic detection activated.")
    else:
        print("Static tsdf mapping activated.")
    print("")
        
    # Run the test
    try:

        nvblox_startup_wait_time_s = 20.0
        print("Starting nvblox")
        nvblox_runner = NvbloxRunner(args.realsense, args.with_humans, args.with_dynamics,
                                     args.model_name, args.model_repository_paths, args.input_binding_name, args.output_binding_name)

        nvblox_runner.launch()
        print(
            f"Waiting for: {nvblox_startup_wait_time_s} for nvblox to start up before launching bag.")
        time.sleep(nvblox_startup_wait_time_s)

        print("Starting the bag")
        bag_launcher = BagLauncher()
        bag_launcher.launch(args.bag_filepath)
        print("Bag done")

        # Getting results (and printing them)
        results_getter = ResultsGetter()
        stamps_dict, cpu_samples, gpu_samples, network_mean_iou_samples, timers_string = results_getter.get_results()

        print(
            f"\n\n{timers_string}\n\nBenchmarking Results\n-------------------------------------\n")
        results_table, results_table_dict = analyze_timestamps.make_results_table(
            stamps_dict, cpu_samples, gpu_samples, network_mean_iou_samples)
        print(results_table)

        # Remove Nan values from dict
        results_table_dict = remove_nan_values_from_dict(
            results_table_dict)

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
            save_results_files(stamps_dict, cpu_samples, gpu_samples, network_mean_iou_samples,
                               timers_string, results_table, output_dir, timestr=timestr)

        # Saving a plot
        if args.save_pulse_plot:
            save_pulse_figure(stamps_dict, output_dir /
                              'pulses.html', args.with_humans)

        # Save results table as json
        if args.save_kpi_json:
            if args.with_humans:
                results_table_dict = {f'{key}_with_humans': value for key, value in results_table_dict.items()}
            if args.save_kpis_in_namespace:
                output_dir = os.path.join(
                    output_dir, args.save_kpis_in_namespace)
            if not os.path.isdir(output_dir):
                os.mkdir(output_dir)
            timings_table_file = os.path.join(
                output_dir, 'ros_performance_kpis.json')
            print(f"Writing the timings table to: {timings_table_file}")
            with open(timings_table_file, "w") as timings_file:
                json.dump(results_table_dict, timings_file, indent=4)

    finally:
        # Kill
        nvblox_runner.kill()
        print("exiting")


if __name__ == '__main__':
    main()
