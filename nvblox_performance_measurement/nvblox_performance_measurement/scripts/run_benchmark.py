#!/usr/bin/env python3

# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

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


def save_isaac_ros_benchmark_results(
        stamps_dict, log_dir, heading, log_file=None, timestr=None):

    # Capturing metrics in the dictionary to be forwarded for json dump.
    performance_data = {}

    def get_performance_data(stamps_ns: np.array, message_name_str: str,
                             messages_name_str: str):
        def ns_to_ms(stamps): return stamps / 1e6
        def ns_to_s(stamps): return stamps / 1e9
        slice_intervals_ms = ns_to_ms(np.diff(stamps_ns))
        if len(slice_intervals_ms) > 0:
            performance_data[f'Number of {messages_name_str} received'] = len(
                stamps_ns)
            performance_data[
                f'First {message_name_str} to {message_name_str} Latency (ms)'] = slice_intervals_ms[0]
            performance_data[f'{message_name_str} Maximum Latency (ms)'] = max(
                slice_intervals_ms)
            performance_data[f'{message_name_str} Minimum Latency (ms)'] = min(
                slice_intervals_ms)
            performance_data[f'{message_name_str} Average Latency (ms)'] = sum(
                slice_intervals_ms) / len(slice_intervals_ms)
            performance_data[f'{message_name_str} Average FPS (s)'] = 1 / \
                ns_to_s((stamps_ns[-1] - stamps_ns[0]) / len(stamps_ns))
        return performance_data

    performance_data.update(
        get_performance_data(
            stamps_dict['/nvblox_node/map_slice'],
            'Slice', 'Slices'))
    performance_data.update(
        get_performance_data(
            stamps_dict['/nvblox_node/depth_processed'],
            'DepthProcessed', 'DepthProcessed'))
    performance_data.update(
        get_performance_data(
            stamps_dict['/nvblox_node/color_processed'],
            'ColorProcessed', 'ColorsProcessed'))
    if '/nvblox_node/mesh' in stamps_dict:
        performance_data.update(
            get_performance_data(
                stamps_dict['/nvblox_node/mesh'],
                'Mesh', 'Meshes'))

    # Dump JSON
    for key, val in performance_data.items():
        performance_data[key] = float(val)
    performance_data['name'] = heading
    if not log_file:
        if timestr is None:
            timestr = time.strftime('%Y%m%d-%H%M%S')
        log_file = os.path.join(log_dir, f'isaac-ros-log-{timestr}.json')
    else:
        log_file = os.path.join(log_dir, log_file)
    print(f'Writing results in Isaac ROS format to: {log_file}')

    with open(log_file, 'x') as f:
        f.write(json.dumps(performance_data))


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
        "--json_file_name",
        help="Exact name of the file where the results will be dumped. If not specified, file name \
            will have the form isaac-ros-log-timestr.json"
    )
    parser.add_argument(
        "--save_json", action='store_const', const=True,
        default=False,
        help="Flag indicating if we should save the isaac ros benchmarking json object.")
    parser.add_argument(
        "--save_results_files", action='store_const', const=True,
        default=False,
        help="Flag indicating if we should save the results to files.")

    args = parser.parse_args()

    # If no bag provided, use the POL bag in {ROS2_WORKSPACE}/isaac_ros_nvblox/nvblox_nav2/...
    # NOTE(alexmillane): Getting the dataset path here is kinda ugly, but I can't see a better way in ROS2
    if args.bag_filepath is None:
        print("No bagfile provided. Using the bag distributed with this package (nvblox_performance_measurement).")
        nvblox_performance_measurement_share_dir = Path(
            get_package_share_directory('nvblox_performance_measurement'))
        workspace_dir = nvblox_performance_measurement_share_dir.parents[3]
        workspace_src_dir = workspace_dir / 'src'
        
        # Fetch git-lfs files
        print(subprocess.getoutput(f'git lfs -X ""'))        
        if args.realsense:
            args.bag_filepath = workspace_src_dir / \
                Path('isaac_ros_nvblox/nvblox_performance_measurement/nvblox_performance_measurement/datasets/realsense_office')
        else:
            args.bag_filepath = workspace_src_dir / \
                Path('isaac_ros_nvblox/nvblox_performance_measurement/nvblox_performance_measurement/datasets/isaac_sim_warehouse')

    # Check the bag exists
    if not os.path.exists(args.bag_filepath):
        sys.exit(f"Bagfile at path: {args.bag_filepath} does not exist.")

    # Run the test
    try:

        nvblox_runner = NvbloxRunner()
        nvblox_runner.launch(args.realsense)

        print("Starting the bag")
        bag_launcher = BagLauncher()
        bag_launcher.launch(args.bag_filepath)
        print("Bag done")

        # Getting results (and printing them)
        results_getter = ResultsGetter()
        stamps_dict, cpu_samples, gpu_samples, timers_string = results_getter.get_results()

        print(
            f"\n\n{timers_string}\n\nBenchmarking Results\n-------------------------------------\n")
        results_table = analyze_timestamps.make_results_table(
            stamps_dict, cpu_samples, gpu_samples)
        print(results_table)

        # Default file output location
        if args.output_dir is None:
            args.output_dir = '/tmp'
        output_dir = Path(args.output_dir)
        timestr = time.strftime('%Y%m%d-%H%M%S')

        # Creating a JSON object used by isaac_ros_benchmarking
        if args.save_json:
            # JSON dump params
            # NOTE(alexmillane): These are copied from isaac_ros-benchmark
            HEADER = 'Nvblox performance metrics'
            LOG_FILE = args.json_file_name
            LOGS_FOLDER = args.output_dir
            save_isaac_ros_benchmark_results(
                stamps_dict, log_dir=LOGS_FOLDER, heading=HEADER,
                log_file=LOG_FILE, timestr=timestr)

        # Saving the collected results
        if args.save_results_files:
            save_results_files(stamps_dict, cpu_samples, gpu_samples,
                               timers_string, results_table, output_dir, timestr=timestr)

    finally:
        # Kill
        nvblox_runner.kill()
        print("exiting")


if __name__ == '__main__':
    main()
