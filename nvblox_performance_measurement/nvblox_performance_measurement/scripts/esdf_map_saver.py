#!/usr/bin/env python3

# Copyright (c) 2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from nvblox_msgs.msg import DistanceMapSlice


class ESDFMapSaver(Node):

    def __init__(self) -> None:
        super().__init__("esdf_map_saver")

        # Params
        self.declare_parameter("file_name")
        self.file_name = (
            self.get_parameter("file_name").get_parameter_value().string_value)

        self.declare_parameter("folder_path")
        self.folder_path = (self.get_parameter(
            "folder_path").get_parameter_value().string_value)

        self.subscription = self.create_subscription(
            DistanceMapSlice,
            "/nvblox_node/static_map_slice",
            self.listener_callback,
            10,
        )

    def listener_callback(self, msg):
        # Process the esdf distance map data
        self.save_esdf_map(msg)

    def save_esdf_map(self, distance_map_msg):
        """Gets the distance map, colors it and saves it in the setup folder

        Args:
            distance_map_msg (DistanceMapSlice): Static distance map output from nvblox
        """
        if distance_map_msg.data:
            distance_map = np.rot90(
                np.flip(
                    np.array(distance_map_msg.data).reshape(
                        (distance_map_msg.height, distance_map_msg.width)),
                    axis=1,
                ),
                k=-1,
            )
            # Mask for known and unknown points
            known_mask = distance_map != distance_map_msg.unknown_value
            unknown_mask = distance_map == distance_map_msg.unknown_value

            # Normalize the known points in the distance map to 0-1
            normalized_map = np.zeros_like(distance_map)
            normalized_map[known_mask] = (distance_map[known_mask] - np.nanmin(
                distance_map[known_mask])) / (
                    np.nanmax(distance_map[known_mask]) -
                    np.nanmin(distance_map[known_mask]))

            # Convert to grayscale (0-255) for known points
            grayscale_map = np.zeros_like(distance_map, dtype=np.uint8)
            grayscale_map[known_mask] = (normalized_map[known_mask] *
                                         255).astype(np.uint8)

            # Apply a colormap (e.g., JET)
            colormap_map = cv2.applyColorMap(grayscale_map, cv2.COLORMAP_JET)

            # Assign background for unknown points
            white_color_background = [255, 255, 255]
            colormap_map[unknown_mask] = white_color_background

            # Save as PNG
            cv2.imwrite(os.path.join(self.folder_path, self.file_name),
                        colormap_map)
        else:
            print("No distance map data yet")


def main():
    rclpy.init()
    esdf_map_saver = ESDFMapSaver()
    rclpy.spin(esdf_map_saver)
    esdf_map_saver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
