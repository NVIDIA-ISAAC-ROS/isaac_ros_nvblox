#!/usr/bin/env python3

# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import sys
import os
import io
import argparse
import numpy as np
import matplotlib.pyplot as plt


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


def plot_pulses(stamps, pulse_height, pulse_color_code='b'):
    pulse_x, pulse_y = generate_pulses(stamps, pulse_height)
    plt.plot(pulse_x, pulse_y, pulse_color_code)


def make_results_table(topic_to_stamps_map, cpu_percentages=None, gpu_percentages=None):
    table_string = ""

    depth_name = 'depth/image'
    color_name = 'color/image'
    lidar_name = 'pointcloud'
    depth_processed_name = '/nvblox_node/depth_processed'
    color_processed_name = '/nvblox_node/color_processed'
    lidar_processed_name = '/nvblox_node/pointcloud_processed'
    slice_name = '/nvblox_node/map_slice'
    mesh_processed_name = '/nvblox_node/mesh_processed'

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
    table_string += f"slice:\t\t\t\t\tprocessed #:\t{len(topic_to_stamps_map[slice_name])}\n"
    table_string += f"mesh:\t\t\t\t\tprocessed #:\t{num_meshes}\n"

    # Message frequencies
    def stamps_to_freq(stamps): return 1e9 / np.mean(np.diff(stamps))
    depth_released_freq = stamps_to_freq(topic_to_stamps_map[depth_name])
    depth_processed_freq = stamps_to_freq(
        topic_to_stamps_map[depth_processed_name])
    color_released_freq = stamps_to_freq(topic_to_stamps_map[color_name])
    color_processed_freq = stamps_to_freq(
        topic_to_stamps_map[color_processed_name])
    lidar_released_freq = stamps_to_freq(topic_to_stamps_map[lidar_name])
    lidar_processed_freq = stamps_to_freq(
        topic_to_stamps_map[lidar_processed_name])
    slice_processed_freq = stamps_to_freq(
        topic_to_stamps_map[slice_name])
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
    table_string += f"slice:\t\t\t\t\tprocessed Hz:\t{slice_processed_freq:0.1f}\n"
    table_string += f"mesh:\t\t\t\t\tprocessed Hz:\t{mesh_processed_freq:0.1f}\n"

    if cpu_percentages is not None:
        table_string += "\n"
        table_string += f"Mean CPU usage: {np.mean(cpu_percentages):0.1f}%\n"
    if gpu_percentages is not None:
        table_string += f"Mean GPU usage: {np.mean(gpu_percentages):0.1f}%\n"

    table_string += "\n\n"

    return table_string


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
