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
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('nvblox_examples_bringup')

    # Launch Arguments
    launch_realsense_arg = DeclareLaunchArgument(
        'launch_realsense', default_value='True',
        description='Whether to launch the realsense driver')
    run_rqt_arg = DeclareLaunchArgument(
        'run_rqt', default_value='True',
        description='Whether to start rqt_image_view')

    # Realsense
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'sensors', 'realsense.launch.py')]),
        condition=IfCondition(LaunchConfiguration('launch_realsense')))

    # Bag recording
    realsense_topics = [
        '/tf_static',
        '/camera/color/camera_info',
        '/camera/color/image_raw',
        '/camera/realsense_splitter_node/output/depth',
        '/camera/depth/camera_info',
        '/camera/realsense_splitter_node/output/infra_1',
        '/camera/infra1/camera_info',
        '/camera/realsense_splitter_node/output/infra_2',
        '/camera/infra2/camera_info'
    ]
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', " ".join(realsense_topics)],
        shell=True, output='screen')

    # Rqt
    rqt_launch = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        condition=IfCondition(LaunchConfiguration('run_rqt')))

    recording_started_msg =\
        '''\n\n\n
        -----------------------------------------------------
                    BAG RECORDING IS STARTING NOW

                 (make sure the realsense node is up)
        -----------------------------------------------------
        \n\n\n'''
    return LaunchDescription([
        launch_realsense_arg,
        run_rqt_arg,

        # Start the realsense and rqt image view
        realsense_launch,
        rqt_launch,

        # Start recording after 10 sec.
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg=recording_started_msg),
                bag_record
            ])])
