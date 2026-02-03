#!/usr/bin/env python3
# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Analyze LiDAR pointcloud from ROS2 bag to calculate nvblox parameters."""

import argparse

import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


def read_pointcloud(bag_path, topic):
    """Read first PointCloud2 message from bag."""
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )
    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, PointCloud2)
            return np.array([[p[0], p[1], p[2]] for p in
                             pc2.read_points(msg,
                                             field_names=('x', 'y', 'z'),
                                             skip_nans=True)])
    raise ValueError(f"Topic '{topic}' not found in bag")


def to_spherical(points):
    """Convert XYZ points to spherical coordinates (Lidar::project in lidar_impl.h)."""
    # Filter valid points
    ranges = np.linalg.norm(points, axis=1)
    valid = ranges > 1e-3
    points, ranges = points[valid], ranges[valid]

    # Spherical coordinates
    elevation = np.arcsin(np.clip(points[:, 2] / ranges, -1.0, 1.0))  # -pi/2 to pi/2
    azimuth = np.arctan2(points[:, 1], points[:, 0])  # -pi to pi

    return ranges, azimuth, elevation, points


def main():
    parser = argparse.ArgumentParser(description='Calculate nvblox LiDAR parameters from ROS2 bag')
    parser.add_argument('bag_path', help='Path to ROS2 bag directory')
    parser.add_argument('--topic',
                        default='/os1_cloud_node/points',
                        help='PointCloud2 topic (default: /os1_cloud_node/points)')
    args = parser.parse_args()

    # Read and process
    print(f'Reading from {args.bag_path}, topic: {args.topic}')
    points = read_pointcloud(args.bag_path, args.topic)
    total_num_points = len(points)
    print(f'Loaded {total_num_points} points')

    ranges, azimuth, elevation, points = to_spherical(points)
    elev_deg = np.rad2deg(elevation)

    # Create bins of 1e-3 degree width only where there is data
    # We are trying to estimate the number of lidar beams and their elevation angles.
    bin_width_deg = 1e-3
    min_elev_deg = np.min(elev_deg)
    max_elev_deg = np.max(elev_deg)

    # Round data to bin width to group points
    elev_deg_rounded = np.round(elev_deg / bin_width_deg) * bin_width_deg
    unique_bins_deg, counts = np.unique(elev_deg_rounded, return_counts=True)

    print(f"\n{'Bin Center (deg)':>18} | {'Bin Center (rad)':>18} | "
          f"{'Count':>8} | {'Min Δ (deg)':>12} | {'Max Δ (deg)':>12} | "
          f"{'Δ to Next (deg)':>15}")
    print('-' * 97)
    for i, (bin_center_deg, count) in enumerate(zip(unique_bins_deg, counts)):
        bin_center_rad = np.deg2rad(bin_center_deg)
        # Indices of points in this bin
        idx = np.abs(elev_deg_rounded - bin_center_deg) < 1e-8  # floating point tolerances
        deviations_deg = elev_deg[idx] - bin_center_deg
        if idx.any():
            min_dev_deg = np.min(deviations_deg)
            max_dev_deg = np.max(deviations_deg)
        else:
            min_dev_deg = max_dev_deg = float('nan')
        # Diff to next bin
        if i < len(unique_bins_deg) - 1:
            diff_to_next = unique_bins_deg[i + 1] - bin_center_deg
        else:
            diff_to_next = float('nan')
        print(f'{bin_center_deg:18.6f} | {bin_center_rad:18.6f} | {count:8d} | '
              f'{min_dev_deg:12.6e} | {max_dev_deg:12.6e} | {diff_to_next:15.6e}')

    print('\n')
    print('Suggested lidar parameters:\n')
    print(f'Number of bins (lidar_height): {len(unique_bins_deg)}')
    points_per_bin = int(total_num_points / len(unique_bins_deg))
    print(f'Number of points per bin (lidar_width): {points_per_bin}')
    if min_elev_deg + max_elev_deg <= 1e-3:
        print('Using equal vertical FoV LiDAR params '
              '(use_non_equal_vertical_fov_lidar_params = False)')
        elev_range_deg = max_elev_deg - min_elev_deg
        elev_range_rad = np.deg2rad(elev_range_deg)
        print(f'Elevation range (lidar_vertical_fov_rad): '
              f'{elev_range_deg:.6f} degrees ({elev_range_rad:.6f} radians)')
    else:
        print('Using non equal vertical FoV LiDAR params '
              '(use_non_equal_vertical_fov_lidar_params = True)')
        min_elev_rad = np.deg2rad(min_elev_deg)
        max_elev_rad = np.deg2rad(max_elev_deg)
        print(f'Min elevation (min_angle_below_zero_elevation_rad): '
              f'{min_elev_deg:.6f} degrees ({min_elev_rad:.6f} radians)')
        print(f'Max elevation (max_angle_above_zero_elevation_rad): '
              f'{max_elev_deg:.6f} degrees ({max_elev_rad:.6f} radians)')

    # Plot histogram of elevation angles (degrees)
    plt.figure(figsize=(10, 6))
    plt.bar(unique_bins_deg, counts, width=bin_width_deg, align='center', edgecolor='k')
    plt.xlabel('Elevation Angle (deg)')
    plt.ylabel('Number of Points')
    plt.title('Elevation Angle Histogram')
    plt.grid(True, linestyle='--', linewidth=0.5, alpha=0.7)
    plt.show()


if __name__ == '__main__':
    main()
