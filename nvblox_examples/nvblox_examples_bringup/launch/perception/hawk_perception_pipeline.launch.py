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

    # NITROS nodes are required to start together to allow format negotiation
    # to take place before all the NITROS nodes are ready to start their underlying GXF graphs.
    # NOTE(remos): For big pipelines the negotiation time
    # can be increased with the type_negotiation_duration_s parameter.
    type_negotiation_duration_s_arg = DeclareLaunchArgument(
        "type_negotiation_duration_s",
        default_value="5",
        description="Duration of the NITROS type negotiation.",
    )
    ess_engine_file_path_arg = DeclareLaunchArgument(
        "ess_engine_file_path",
        default_value="/workspaces/isaac_ros-dev/models/ess.engine",
        description="Path to the ESS engine file.",
    )
    flatten_odometry_to_2d_arg = DeclareLaunchArgument(
        "flatten_odometry_to_2d",
        default_value="True",
        description=
        "Whether to flatten the odometry to 2D (camera only moving on XY-plane).",
    )

    # Option to attach the nodes to a shared component container for speed ups through intra process communication.
    # Make sure to set the 'component_container_name' to the name of the component container you want to attach to.
    attach_to_shared_component_container_arg = LaunchConfiguration(
        'attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration(
        'component_container_name', default='hawk_perception_pipeline')
    global_frame = LaunchConfiguration("global_frame", default="odom")

    # If we do not attach to a shared component container we have to create our own container.
    hawk_perception_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=UnlessCondition(attach_to_shared_component_container_arg))

    # Hawk processing
    hawk_processing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, "launch", "sensors",
                         "hawk_processing.launch.py")
        ]),
        launch_arguments={
            "attach_to_shared_component_container":
            "True",
            "type_negotiation_duration_s":
            LaunchConfiguration("type_negotiation_duration_s"),
            "component_container_name":
            component_container_name_arg,
        }.items(),
    )

    # ESS
    ess_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, "launch", "perception", "ess.launch.py")
        ]),
        launch_arguments={
            "attach_to_shared_component_container":
            "True",
            "type_negotiation_duration_s":
            LaunchConfiguration("type_negotiation_duration_s"),
            "engine_file_path":
            LaunchConfiguration("ess_engine_file_path"),
            "component_container_name":
            component_container_name_arg,
        }.items(),
    )

    # Vslam
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, "launch", "perception",
                         "vslam.launch.py")
        ]),
        launch_arguments={
            "output_odom_frame_name":
            global_frame,
            "run_odometry_flattening":
            LaunchConfiguration("flatten_odometry_to_2d"),
            "setup_for_hawk":
            "True",
            "attach_to_shared_component_container":
            "True",
            "component_container_name":
            component_container_name_arg,
        }.items(),
    )

    # Nvblox
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, "launch", "nvblox", "nvblox.launch.py")
        ]),
        launch_arguments={
            "global_frame": global_frame,
            "attach_to_shared_component_container": "True",
            "setup_for_hawk": "True",
            "component_container_name":
            component_container_name_arg,
        }.items(),
    )

    return LaunchDescription([
        hawk_perception_container,
        type_negotiation_duration_s_arg, ess_engine_file_path_arg,
        flatten_odometry_to_2d_arg,
        hawk_processing_launch, ess_launch, vslam_launch,
        nvblox_launch,
    ])
