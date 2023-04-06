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
import numpy as np
from typing import Dict, Tuple

import message_filters
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from nvblox_msgs.msg import SemanticLabelsStamped
from sensor_msgs.msg import Image


class SemanticConverter(Node):
    def __init__(self) -> None:
        '''
        Helper node to convert IsaacSim Semantic Labels to a consistent semantic segmentation image
        '''
        super().__init__('semantic_label_converter')

        # Declare params
        self.declare_parameter('camera_1_enabled', True)
        self.declare_parameter('camera_2_enabled', True)
        self.declare_parameter('camera_1_name', 'left')
        self.declare_parameter('camera_2_name', 'right')
        self.declare_parameter('labels.names', ['unlabelled', 'person'])

        # Get params
        camera_1_enabled = self.get_parameter(
            'camera_1_enabled').get_parameter_value().bool_value
        camera_2_enabled = self.get_parameter(
            'camera_2_enabled').get_parameter_value().bool_value
        camera_1_name = self.get_parameter(
            'camera_1_name').get_parameter_value().string_value
        camera_2_name = self.get_parameter(
            'camera_2_name').get_parameter_value().string_value
        label_names = self.get_parameter(
            'labels.names').get_parameter_value().string_array_value

        # Get dictionary that defines
        # how label names are converted to ids and color
        self.label_conversion_dict = {}
        for label_name in label_names:
            output_id_param_name = f"labels.{label_name}.output_id"
            output_color_param_name = f"labels.{label_name}.output_color"

            self.declare_parameter(output_id_param_name, 0)
            self.declare_parameter(output_color_param_name, [0, 0, 0])

            output_id = self.get_parameter(
                output_id_param_name).get_parameter_value().integer_value
            output_color = list(self.get_parameter(
                output_color_param_name).get_parameter_value().integer_array_value)

            self.label_conversion_dict[label_name] = {
                'output_id': output_id,
                'output_color': output_color
            }

        if camera_1_enabled:
            self.init_camera(camera_1_name)
        if camera_2_enabled:
            self.init_camera(camera_2_name)

        self.bridge = CvBridge()

    def init_camera(self, camera_name: str) -> None:
        '''
        Initialize publishers and subscribers for a camera
        Args:
            camera_name (str): The name of the camera
        '''
        # Subscriber
        image_subscriber = message_filters.Subscriber(
            self, Image, f"/{camera_name}/semantic")
        labels_subscriber = message_filters.Subscriber(
            self, SemanticLabelsStamped,
            f"/semantic_conversion/{camera_name}/labels_stamped")

        # Publisher
        publisher_mono8 = self.create_publisher(
            Image, f"/semantic_conversion/{camera_name}/semantic_mono8", 1)
        publisher_colorized = self.create_publisher(
            Image, f"/semantic_conversion/{camera_name}/semantic_colorized", 1)

        # Synchronized callback
        def on_camera_image_received(image_msg, label_msg): return \
            self.on_image_received(
                publisher_mono8, publisher_colorized, image_msg, label_msg)
        ts = message_filters.TimeSynchronizer(
            [image_subscriber, labels_subscriber], 10)
        ts.registerCallback(on_camera_image_received)

    def on_image_received(self, publisher_mono8, publisher_colorized, image_msg: Image,
                          labels_msg: SemanticLabelsStamped) -> None:
        '''
        Callback to convert semantic image from IsaacSim to a consistent label image in mono8

        Args:
            image_msg (Image): Input image from Isaacsim, is in CV16SC1 format and labels id vary with scene
            labels_msg (SemanticLabelsStamped): Stamped input labels message for the current image.
        '''
        # Load the labels as a json
        labels_dict = json.loads(labels_msg.labels)
        # Build LUT for color conversions
        lut_labels, lut_colors = self.build_labels_lut(labels_dict)
        shape = (image_msg.height, image_msg.width, 1)
        # Convert from signed int 32 to unsigned char (will cause issues if more than 255 classes)
        data = np.frombuffer(image_msg.data, dtype=np.int32).reshape(shape)
        data_mono8 = data.astype(np.uint8)

        # Generate labels arrays with the lookups (for id and coloring)
        try:
            labels = np.take(lut_labels, data_mono8)
            colors = np.take(lut_colors, data_mono8, 0).squeeze()
        except Exception as e:
            print("Exception raised in semantic label converter:\n", e)
            print("WARNING: We will skip this semantic image and label pair.")
            return

        # Convert back to image messages and publish
        labels_msg = self.bridge.cv2_to_imgmsg(labels, 'mono8')
        labels_msg.header = image_msg.header
        colors_msg = self.bridge.cv2_to_imgmsg(colors, "rgb8")
        colors_msg.header = image_msg.header
        publisher_mono8.publish(labels_msg)
        publisher_colorized.publish(colors_msg)

    def build_labels_lut(
        self, current_labels: Dict[str,
                                   Dict[str,
                                        str]]) -> Tuple[np.array, np.array]:
        '''
        Build labels lookup table (LUT) from current labels dictionary. The dictionary is formatted as
        {<class_id_0>: {"class": <class_name_x>}} where class_id_x is the id of the class in the
        image and class_name_x is the one that was entered in the semantics schema. This lookup
        maps all classes that are not in the reference as unlabelled, and remaps ids / colors of the
        reference classes to the desired reference one

        Args:
            current_labels (Dict[str, Dict[str, str]]): Labels string coming from IsaacSim

        Returns:
            Tuple[np.array, np.array]: LUT for ids (Nx1) and colors (Nx3) where N is the number
            of classes
        '''
        # First, get the maximum label that appears in the image
        max_label = -1
        for label_id, _ in current_labels.items():
            label_id_int = int(label_id)
            if label_id_int > max_label:
                max_label = label_id_int
        # Initialize all remappings to zero
        lut_labels = np.zeros(max_label + 1, dtype=np.uint8)
        lut_colors = np.zeros((max_label + 1, 3), dtype=np.uint8)
        # Go over all labels that are present, and get their remap
        for label_id, label_name_dict in current_labels.items():
            label_id_int = int(label_id)
            label_name = label_name_dict.get("class", None)
            if label_name is None:
                continue
            label_name = label_name.lower()
            # Look for the output id / color if if exists
            target_label = self.label_conversion_dict.get(
                label_name, {}).get("output_id", 0)
            target_color = self.label_conversion_dict.get(label_name, {}).get(
                "output_color", [0, 0, 0])
            # Update the remap
            lut_labels[label_id_int] = target_label
            lut_colors[label_id_int] = target_color
        return lut_labels, lut_colors


def main():
    rclpy.init()
    converter = SemanticConverter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
