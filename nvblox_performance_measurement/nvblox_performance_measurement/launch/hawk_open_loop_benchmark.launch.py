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
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch_ros.actions import (SetParameter, SetParametersFromFile, SetRemap)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    bringup_dir = get_package_share_directory("nvblox_examples_bringup")

    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Omniverse Isaac Sim) clock if true",
    )
    run_rviz_arg = DeclareLaunchArgument("run_rviz",
                                         default_value="True",
                                         description="Whether to start RVIZ")
    run_nvblox_perf_wrapper_arg = DeclareLaunchArgument(
        "run_nvblox_perf_wrapper",
        default_value="True",
        description="Whether to start nvblox performance node",
    )
    use_rectify_for_vslam_arg = DeclareLaunchArgument(
        "use_rectify_for_vslam",
        default_value="True",
        description="Whether to run vslam on the rectified images or not.",
    )
    flatten_odometry_to_2d_arg = DeclareLaunchArgument(
        "flatten_odometry_to_2d",
        default_value="True",
        description=
        "Whether to flatten the odometry to 2D (camera only moving on XY-plane).",
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
    global_frame = LaunchConfiguration("global_frame", default="odom")
    mapping_type_arg = DeclareLaunchArgument(
        "mapping_type",
        default_value="static_tsdf",
        description="Mapping type to choose between dynamic and static tsdf",
    )
    timing_sampling_rate_arg = DeclareLaunchArgument(
        "timing_sampling_rate_ms",
        default_value="1000",
        description=
        "The sampling rate of the nvblox timers/rates for this benchmark",
    )
    nvblox_param_dir_arg = DeclareLaunchArgument(
        "nvblox_params_file",
        default_value=os.path.join(bringup_dir, "config", "nvblox",
                                   "nvblox_base.yaml"),
    )
    nvblox_param_dir_spec_arg = DeclareLaunchArgument(
        "nvblox_params_spec_file",
        default_value=os.path.join(bringup_dir, "config", "nvblox",
                                   "specializations", "nvblox_hawk.yaml"),
    )

    # Remaps
    hawk_remaps = [
        ("depth/image", "/front_stereo_camera/depth"),
        ("depth/camera_info", "/front_stereo_camera/left/camera_info_resized"),
        ("color/image", "/front_stereo_camera/left/image_resized"),
        ("color/camera_info", "/front_stereo_camera/left/camera_info_resized"),
        ("pointcloud", "/point_cloud"),
    ]

    nvblox_remaps = [
        ("/nvblox_human_node/color_processed", "/nvblox_node/color_processed"),
        ("/nvblox_human_node/depth_processed", "/nvblox_node/depth_processed"),
        (
            "/nvblox_human_node/pointcloud_processed",
            "/nvblox_node/pointcloud_processed",
        ),
        ("/nvblox_human_node/static_map_slice",
         "/nvblox_node/static_map_slice"),
        ("/nvblox_human_node/mesh_processed", "/nvblox_node/mesh_processed"),
    ]

    # Create a shared container to hold composable nodes
    # for speed ups through intra process communication.
    shared_container_name = "shared_container"
    shared_container = ComposableNodeContainer(
        name=shared_container_name,
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
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

    # NOTE(alexmillane): We use this group action to reach down through the launchfile
    # heirachy and set an nvblox parameter (in this case the parameter controlling
    # how aften timing information is output).
    hawk_perception_action = GroupAction([
        SetParameter(name='print_statistics_on_console_period_ms',
                     value=LaunchConfiguration('timing_sampling_rate_ms',
                                               default=10000)),
        hawk_perception_launch
    ])

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

    # Performance measurements
    hawk_recorder_node = Node(
        package="nvblox_performance_measurement",
        executable="results_collector_node",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
        output="screen",
        remappings=hawk_remaps + nvblox_remaps,
    )

    cpu_usage_node = Node(
        package="nvblox_cpu_gpu_tools",
        executable="cpu_percentage_node",
        parameters=[{
            "node_process_name": "component_container_mt"
        }],
        output="screen",
    )

    gpu_usage_node = Node(
        package="nvblox_cpu_gpu_tools",
        executable="gpu_percentage_node",
        parameters=[],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg, run_rviz_arg, run_nvblox_perf_wrapper_arg,
        mapping_type_arg, nvblox_param_dir_arg, nvblox_param_dir_spec_arg,
        ess_engine_file_path_arg, type_negotiation_duration_s_arg,
        use_rectify_for_vslam_arg, flatten_odometry_to_2d_arg,
        timing_sampling_rate_arg, shared_container, hawk_perception_action,
        hawk_recorder_node, cpu_usage_node, gpu_usage_node, rviz_launch
    ])
