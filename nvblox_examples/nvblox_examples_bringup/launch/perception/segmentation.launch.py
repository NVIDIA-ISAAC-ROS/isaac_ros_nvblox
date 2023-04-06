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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap, Node


def generate_launch_description():

    segnet_bringup_dir = get_package_share_directory('isaac_ros_unet')

    # Semantic segmentation
    segnet_launch = GroupAction([
        SetRemap(src=['image'],
                 dst=['/camera/color/image_raw_padded']),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(segnet_bringup_dir,
                             'launch', 'isaac_ros_unet_triton.launch.py')),
            launch_arguments={'model_name': 'peoplesemsegnet',
                              'model_repository_paths': "['/workspaces/isaac_ros-dev/models']",
                              'input_binding_names': "['input_1:0']",
                              'output_binding_names': "['argmax_1']",
                              'network_output_type': 'argmax'}.items())
    ])

    image_padding_node = Node(
        name='image_padding_node',
        package='nvblox_image_padding',
        executable='image_padding_cropping_node',
        parameters=[{'image_qos': 'SYSTEM_DEFAULT',
                     'desired_height': 544,
                     'desired_width': 960
                     }],
        remappings=[('~/image_in', '/camera/color/image_raw'),
                    ('~/image_out', '/camera/color/image_raw_padded')])

    image_cropping_node = Node(
        name='image_cropping_node',
        package='nvblox_image_padding',
        executable='image_padding_cropping_node',
        parameters=[{'image_qos': 'SYSTEM_DEFAULT',
                     'desired_height': 480,
                     'desired_width': 640
                     }],
        remappings=[('~/image_in', '/unet/raw_segmentation_mask'),
                    ('~/image_out', '/unet/raw_segmentation_mask_depadded')])

    return LaunchDescription([
        segnet_launch,
        image_padding_node,
        image_cropping_node])
