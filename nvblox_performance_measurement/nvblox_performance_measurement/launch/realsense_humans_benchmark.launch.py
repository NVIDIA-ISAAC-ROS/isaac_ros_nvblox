# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    examples_bringup_dir = get_package_share_directory(
        'nvblox_examples_bringup')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation (Omniverse Isaac Sim) clock if true')
    model_name = LaunchConfiguration(
        'model_name', default='peoplesemsegnet')
    model_repository_paths = LaunchConfiguration(
        'model_repository_paths', default="['/workspaces/isaac_ros-dev/models']")
    input_binding_names = LaunchConfiguration(
        'input_binding_names', default="['input_1:0']")
    output_binding_names = LaunchConfiguration(
        'output_binding_names', default="['argmax_1']")

    nvblox_param_dir_arg = DeclareLaunchArgument(
        'nvblox_params_file',
        default_value=os.path.join(
            get_package_share_directory(
                'nvblox_examples_bringup'), 'config', 'nvblox', 'nvblox_base.yaml'
        ),
    )

    nvblox_human_param_dir_arg = DeclareLaunchArgument(
        'nvblox_human_params_file',
        default_value=os.path.join(
            get_package_share_directory(
                'nvblox_examples_bringup'), 'config', 'nvblox', 'specializations', 'nvblox_humans.yaml'
        ),
    )

    # Remaps
    realsense_remaps = [('depth/image', '/camera/realsense_splitter_node/output/depth'),
                        ('depth/camera_info', '/camera/depth/camera_info'),
                        ('color/image', '/camera/color/image_raw'),
                        ('color/camera_info', '/camera/color/camera_info'),
                        ('mask/image', '/unet/raw_segmentation_mask_depadded'),
                        ('mask/camera_info', '/camera/color/camera_info')]

    # Nvblox node + Results recorder - realsense
    realsense_recorder_node = Node(
        package='nvblox_performance_measurement',
        executable='results_collector_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        remappings=realsense_remaps
    )

    # Create a shared container to hold composable nodes 
    # for speed ups through intra process communication.
    shared_container_name = "shared_nvblox_container"
    shared_container = Node(
        name=shared_container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen')

    # Nvblox performance measurement node
    nvblox_node = LoadComposableNodes(
        target_container=shared_container_name,
        composable_node_descriptions=[
        ComposableNode(
        name='nvblox_human_node',
        package='nvblox_performance_measurement',
        plugin='nvblox::NvbloxHumanPerformanceMeasurementNode',
        parameters=[LaunchConfiguration('nvblox_params_file'),
                    LaunchConfiguration('nvblox_human_params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'depth_qos': 'SENSOR_DATA',
                     'color_qos': 'SENSOR_DATA'}],
        remappings=realsense_remaps)])

    # Segmentation
    segmentation_launch_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            examples_bringup_dir, 'launch', 'perception', 'segmentation.launch.py')]),
        launch_arguments={
            'model_name': model_name, 
            'model_repository_paths': model_repository_paths, 
            'input_binding_names': input_binding_names,
            'output_binding_names': output_binding_names, 
            'attach_to_shared_component_container': 'True',
            'component_container_name': shared_container_name}.items())

    cpu_usage_node = Node(
        package='nvblox_cpu_gpu_tools', executable='cpu_percentage_node',
        parameters=[{
            'node_process_name':
            'component_container_mt'}],
        output='screen')

    gpu_usage_node = Node(
        package='nvblox_cpu_gpu_tools', executable='gpu_percentage_node',
        parameters=[],
        output='screen')

    return LaunchDescription([use_sim_time_arg,
                              nvblox_param_dir_arg,
                              nvblox_human_param_dir_arg,
                              realsense_recorder_node,
                              shared_container,
                              nvblox_node,
                              segmentation_launch_realsense,
                              cpu_usage_node,
                              gpu_usage_node
                              ])
