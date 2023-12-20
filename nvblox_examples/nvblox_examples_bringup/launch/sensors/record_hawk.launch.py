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
    launch_hawk_arg = DeclareLaunchArgument(
        'launch_hawk',
        default_value='True',
        description='Whether to launch the hawk driver')
    run_rqt_arg = DeclareLaunchArgument(
        'run_rqt',
        default_value='False',
        description='Whether to start rqt_image_view')

    # Hawk
    hawk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, 'launch', 'sensors',
                         'hawk_driver.launch.py')
        ]),
        condition=IfCondition(LaunchConfiguration('launch_hawk')))

    # Bag recording
    hawk_topics = [
        '/tf',
        '/tf_static',
        '/hawk/left/image_raw',
        '/hawk/left/camera_info',
        '/hawk/right/image_raw',
        '/hawk/right/camera_info',
        # Topics for visual slam debugging:
        '/hawk/imu',
        'visual_slam/tracking/odometry',
        'visual_slam/tracking/vo_pose',
        'visual_slam/status'
    ]
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', " ".join(hawk_topics)],
        shell=True,
        output='screen')

    # Rqt
    rqt_launch = Node(package='rqt_image_view',
                      executable='rqt_image_view',
                      name='rqt_image_view',
                      condition=IfCondition(LaunchConfiguration('run_rqt')))

    recording_started_msg =\
        '''\n\n\n
        -----------------------------------------------------
                    BAG RECORDING IS STARTING NOW

                 (make sure the hawk node is up)
        -----------------------------------------------------
        \n\n\n'''
    return LaunchDescription([
        launch_hawk_arg,
        run_rqt_arg,

        # Start the hawk and rqt image view
        hawk_launch,
        rqt_launch,

        # Start recording after 10 sec.
        TimerAction(period=10.0,
                    actions=[LogInfo(msg=recording_started_msg), bag_record])
    ])
