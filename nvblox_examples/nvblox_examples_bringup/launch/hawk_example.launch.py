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
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("nvblox_examples_bringup")

    # Launch Arguments
    run_rviz_arg = DeclareLaunchArgument("run_rviz",
                                         default_value="True",
                                         description="Whether to start RVIZ")
    from_bag_arg = DeclareLaunchArgument(
        "from_bag",
        default_value="False",
        description="Whether to run from a bag or live realsense data",
    )
    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        default_value="rosbag2*",
        description="Path of the bag (only used if from_bag == True)",
    )
    flatten_odometry_to_2d_arg = DeclareLaunchArgument(
        "flatten_odometry_to_2d",
        default_value="True",
        description=
        "Whether to flatten the odometry to 2D (camera only moving on XY-plane).",
    )
    record_arg = DeclareLaunchArgument(
        "record",
        default_value="False",
        description="Record images to play back offline.",
    )
    ess_engine_file_path_arg = DeclareLaunchArgument(
        "ess_engine_file_path",
        default_value="/workspaces/isaac_ros-dev/models/ess.engine",
        description="Path to the ESS engine file.",
    )
    type_negotiation_duration_s_arg = DeclareLaunchArgument(
        "type_negotiation_duration_s",
        default_value="5",
        description="Duration of the NITROS type negotiation.",
    )
    hawk_module_id_arg = DeclareLaunchArgument(
        "hawk_module_id",
        default_value="5",
        description="ID of the hawk camera to use.")
    global_frame = LaunchConfiguration("global_frame", default="odom")
    bag_play_args = DeclareLaunchArgument(
        "bag_play_args",
        default_value="",
        description='A (quotation bounded) list of bag replay arguments. E.g.:'
        'bag_play_args:="--remap /old_topic:=/new_topic --qos-profile-overrides-path qos_overrides.yaml"'
    )

    # Create a shared container to hold composable nodes
    # for speed ups through intra process communication.
    shared_container_name = "shared_container"
    shared_container = Node(
        name=shared_container_name,
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
    )

    # Hawk driver
    hawk_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, "launch", "sensors",
                         "hawk_driver.launch.py")
        ]),
        launch_arguments={
            "attach_to_shared_component_container":
            "True",
            "type_negotiation_duration_s":
            LaunchConfiguration("type_negotiation_duration_s"),
            "hawk_module_id":
            LaunchConfiguration("hawk_module_id"),
            "component_container_name":
            shared_container_name,
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("from_bag")),
    )

    # Hawk perception pipeline (hawk processing, VSLAM, nvblox, ESS)
    hawk_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, "launch", "perception",
                         "hawk_perception_pipeline.launch.py")
        ]),
        launch_arguments={
            "attach_to_shared_component_container":
            "True",
            "type_negotiation_duration_s":
            LaunchConfiguration("type_negotiation_duration_s"),
            "component_container_name":
            shared_container_name,
            "ess_engine_file_path":
            LaunchConfiguration("ess_engine_file_path"),
            "flatten_odometry_to_2d":
            LaunchConfiguration("flatten_odometry_to_2d"),
        }.items(),
    )

    # Record
    record_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, "launch", "sensors",
                         "record_hawk.launch.py")
        ]),
        launch_arguments={"launch_hawk": "False"}.items(),
        condition=IfCondition(LaunchConfiguration("record")),
    )

    # Ros2 bag
    bag_play = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play -l",
            LaunchConfiguration("bag_path"),
            LaunchConfiguration("bag_play_args"),
        ],
        shell=True,
        output="screen",
        condition=IfCondition(LaunchConfiguration("from_bag")),
    )

    # Rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, "launch", "visualization",
                         "rviz.launch.py")
        ]),
        launch_arguments={
            "config_name": "hawk_example.rviz",
            "global_frame": global_frame,
        }.items(),
        condition=IfCondition(LaunchConfiguration("run_rviz")),
    )

    # Decoders.
    # NOTE(alexmillane): For now we always launch these if we're running from a bag.
    decoders_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, "launch", "sensors",
                         "hawk_decoding.launch.py")
        ]),
        launch_arguments={
            "type_negotiation_duration_s":
            LaunchConfiguration("type_negotiation_duration_s"),
            "attach_to_shared_component_container": "True",
            "component_container_name": shared_container_name,
        }.items(),
        condition=IfCondition(LaunchConfiguration("from_bag")),
    )

    # Foxglove
    foxglove_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'visualization', 'foxglove_bridge.launch.py')]),
        launch_arguments={'send_buffer_limit': '10000000'}.items()) # 10 mega bytes

    # Image Resize to 96*60 (for streaming and visualization on host computer)
    image_resize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'visualization', 'image_resize_for_visualization.launch.py')]),
        launch_arguments={
            'attach_to_shared_component_container': 'True',
            'type_negotiation_duration_s': LaunchConfiguration('type_negotiation_duration_s'),
            'component_container_name': shared_container_name}.items())

    return LaunchDescription([
        run_rviz_arg, from_bag_arg, bag_path_arg, record_arg,
        flatten_odometry_to_2d_arg, ess_engine_file_path_arg,
        type_negotiation_duration_s_arg, hawk_module_id_arg, shared_container,
        hawk_driver_launch, hawk_perception_launch,
        record_launch, bag_play_args, bag_play,
        decoders_launch, rviz_launch, foxglove_bridge_launch,
        image_resize_launch
    ])
