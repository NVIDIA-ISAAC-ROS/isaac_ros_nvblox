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

import sys
import os
import argparse
import numpy as np


def generate_pulses(stamps, pulse_height):
    pulse_x = []
    pulse_y = []
    pulse_half_width_ns = 1000
    for t in stamps:
        pulse_x.append(t - pulse_half_width_ns)
        pulse_x.append(t)
        pulse_x.append(t + pulse_half_width_ns)
        pulse_y.append(0)
        pulse_y.append(pulse_height)
        pulse_y.append(0)
    return np.array(pulse_x), np.array(pulse_y)


def get_pulses_plot(stamps, pulse_height, line_name):
    import plotly.graph_objects as go
    pulse_x, pulse_y = generate_pulses(stamps, pulse_height)
    return go.Scatter(x=pulse_x, y=pulse_y, name=line_name)


def filter_out_upper_lower_percentile(array, upper_percentile: float = 90, lower_percentile: float = 10):
    assert(upper_percentile > 0 and upper_percentile < 100)
    assert(lower_percentile > 0 and lower_percentile < 100)
    assert(lower_percentile < upper_percentile)
    percentile_upper = np.percentile(array, upper_percentile)
    percentile_lower = np.percentile(array, lower_percentile)
    keep_flags = np.logical_and(
        (array < percentile_upper), (array > percentile_lower))
    return array[keep_flags]


def make_results_table(topic_to_stamps_map, cpu_percentages=None, gpu_percentages=None, network_performance=None):
    table_string = ""

    depth_name = 'depth/image'
    color_name = 'color/image'
    sem_name = '/unet/raw_segmentation_mask'
    lidar_name = 'pointcloud'
    depth_processed_name = '/nvblox_human_node/depth_processed'
    color_processed_name = '/nvblox_human_node/color_processed'
    lidar_processed_name = '/nvblox_human_node/pointcloud_processed'
    base_slice_name = '/nvblox_human_node/static_map_slice'
    human_slice_name = '/nvblox_human_node/dynamic_map_slice'
    mesh_processed_name = '/nvblox_human_node/mesh_processed'

    # Message numbers
    if mesh_processed_name in topic_to_stamps_map:
        num_meshes = len(topic_to_stamps_map[mesh_processed_name])
    else:
        num_meshes = 0

    table_string += "\n"
    table_string += "Message Numbers\n"
    table_string += f"depth:\t\treleased #:\t{len(topic_to_stamps_map[depth_name])}\tprocessed #:\t{len(topic_to_stamps_map[depth_processed_name])}\n"
    table_string += f"color:\t\treleased #:\t{len(topic_to_stamps_map[color_name])}\tprocessed #:\t{len(topic_to_stamps_map[color_processed_name])}\n"
    table_string += f"lidar:\t\treleased #:\t{len(topic_to_stamps_map[lidar_name])}\tprocessed #:\t{len(topic_to_stamps_map[lidar_processed_name])}\n"
    table_string += f"semantics:\t\t\t\tprocessed #:\t{len(topic_to_stamps_map[sem_name])}\n"
    table_string += f"base slice:\t\t\t\tprocessed #:\t{len(topic_to_stamps_map[base_slice_name])}\n"
    table_string += f"human slice:\t\t\t\tprocessed #:\t{len(topic_to_stamps_map[human_slice_name])}\n"
    table_string += f"mesh:\t\t\t\t\tprocessed #:\t{num_meshes}\n"

    # Message frequencies
    def stamps_to_freq(stamps): return 1e9 / np.mean(np.diff(stamps))
    depth_released_freq = stamps_to_freq(topic_to_stamps_map[depth_name])
    depth_processed_freq = stamps_to_freq(
        topic_to_stamps_map[depth_processed_name])
    color_released_freq = stamps_to_freq(topic_to_stamps_map[color_name])
    color_processed_freq = stamps_to_freq(
        topic_to_stamps_map[color_processed_name])
    semantic_processed_freq = stamps_to_freq(
        topic_to_stamps_map[sem_name])
    lidar_released_freq = stamps_to_freq(topic_to_stamps_map[lidar_name])
    lidar_processed_freq = stamps_to_freq(
        topic_to_stamps_map[lidar_processed_name])
    base_slice_processed_freq = stamps_to_freq(
        topic_to_stamps_map[base_slice_name])
    human_slice_processed_freq = stamps_to_freq(
        topic_to_stamps_map[human_slice_name])
    if mesh_processed_name in topic_to_stamps_map:
        mesh_processed_freq = stamps_to_freq(
            topic_to_stamps_map[mesh_processed_name])
    else:
        mesh_processed_freq = 0

    table_string += "\n"
    table_string += "Message Frequencies\n"
    table_string += f"depth:\t\treleased Hz:\t{depth_released_freq:0.1f}\tprocessed Hz:\t{depth_processed_freq:0.1f}\n"
    table_string += f"color:\t\treleased Hz:\t{color_released_freq:0.1f}\tprocessed Hz:\t{color_processed_freq:0.1f}\n"
    table_string += f"lidar:\t\treleased Hz:\t{lidar_released_freq:0.1f}\tprocessed Hz:\t{lidar_processed_freq:0.1f}\n"
    table_string += f"semantics:\t\t\t\tprocessed Hz:\t{semantic_processed_freq:0.1f}\n"
    table_string += f"base slice:\t\t\t\tprocessed Hz:\t{base_slice_processed_freq:0.1f}\n"
    table_string += f"human slice:\t\t\t\tprocessed Hz:\t{human_slice_processed_freq:0.1f}\n"
    table_string += f"mesh:\t\t\t\t\tprocessed Hz:\t{mesh_processed_freq:0.1f}\n"

    # Putting the table into a dictionary
    # Use different names when running with or without humans
    table_dict = {'depth_released_freq': depth_released_freq,
                  'depth_processed_freq': depth_processed_freq,
                  'color_released_freq': color_released_freq,
                  'color_processed_freq': color_processed_freq,
                  'semantic_processed_freq': semantic_processed_freq,
                  'pointcloud_released_freq': lidar_released_freq,
                  'pointcloud_processed_freq': lidar_processed_freq}

    if cpu_percentages is not None:
        table_string += "\n"
        cpu_mean = np.mean(filter_out_upper_lower_percentile(cpu_percentages))
        table_string += f"Mean CPU usage: {cpu_mean:0.1f}%\n"
        table_dict['cpu_usage'] = cpu_mean
    if gpu_percentages is not None:
        gpu_mean = np.mean(filter_out_upper_lower_percentile(gpu_percentages))
        table_string += f"Mean GPU usage: {gpu_mean:0.1f}%\n"
        table_dict['gpu_usage'] = gpu_mean
    if network_performance is not None:
        table_string += f"Network mIoU: {(100*np.mean(network_performance)):0.1f}%\n"
        table_dict['network_miou'] = 100*np.mean(network_performance)

    table_string += "\n\n"

    return table_string, table_dict


def main():

    parser = argparse.ArgumentParser(
        description="Extract statistics from message timestamps.")
    parser.add_argument("path", metavar="path", type=str,
                        help="Path to the timestamps file to use.")
    args = parser.parse_args()
    if not os.path.isfile(args.path):
        sys.exit(f"Timstamps file: {args.path} does not exist.")

    # Load from npz file.
    topic_to_stamps_map = np.load(args.path, allow_pickle=True).item()

    print(make_results_table(topic_to_stamps_map))


if __name__ == '__main__':
    main()
