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
from launch_ros.actions import Node


def generate_launch_description():

    # Config file
    config_file = os.path.join(
        get_package_share_directory('semantic_label_conversion'),
        'params', 'semantic_label_conversion.yaml')

    semantic_label_stamper = Node(package='semantic_label_conversion',
                                  executable='semantic_label_stamper.py',
                                  name='semantic_label_stamper',
                                  output='screen',
                                  emulate_tty=True,
                                  parameters=[config_file])

    semantic_label_converter = Node(package='semantic_label_conversion',
                                    executable='semantic_label_converter.py',
                                    name='semantic_label_converter',
                                    output='screen',
                                    emulate_tty=True,
                                    parameters=[config_file])

    return LaunchDescription([
        semantic_label_stamper,
        semantic_label_converter
    ])
