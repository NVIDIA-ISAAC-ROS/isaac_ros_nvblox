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
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition


def generate_launch_description():
    examples_bringup_dir = get_package_share_directory(
        'nvblox_examples_bringup')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation (Omniverse Isaac Sim) clock if true')
    mapping_type_arg = DeclareLaunchArgument(
        'mapping_type', default_value='static_tsdf',
        description='Mapping type to choose between dynamic and static tsdf')

    nvblox_param_dir_arg = DeclareLaunchArgument(
        'nvblox_params_file',
        default_value=os.path.join(
            examples_bringup_dir, 'config', 'nvblox', 'nvblox_base.yaml'
        ),
    )

    # Remaps
    isaac_sim_remaps = [('depth/image', '/front/stereo_camera/right/depth'),
                        ('depth/camera_info', '/front/stereo_camera/right/camera_info'),
                        ('color/image', '/front/stereo_camera/left/rgb'),
                        ('color/camera_info', '/front/stereo_camera/left/camera_info'),
                        ('pointcloud', '/point_cloud'),
                        ('mask/image', '/unet/raw_segmentation_mask_depadded'),
                        ('mask/camera_info', '/front/stereo_camera/left/camera_info')]

    nvblox_remaps = [('/nvblox_human_node/color_processed', '/nvblox_node/color_processed'),
                     ('/nvblox_human_node/depth_processed',
                      '/nvblox_node/depth_processed'),
                     ('/nvblox_human_node/pointcloud_processed',
                      '/nvblox_node/pointcloud_processed'),
                     ('/nvblox_human_node/static_map_slice', '/nvblox_node/static_map_slice'),
                     ('/nvblox_human_node/mesh_processed', '/nvblox_node/mesh_processed')]

    # Nvblox node + Results recorder - Sim
    sim_recorder_node = Node(
        package='nvblox_performance_measurement',
        executable='results_collector_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        remappings=isaac_sim_remaps + nvblox_remaps
    )

    # Nvblox performance measurement node
    nvblox_node = ComposableNode(
            name='nvblox_node',
            package='nvblox_performance_measurement',
            plugin='nvblox::NvbloxPerformanceMeasurementNode',
            remappings=isaac_sim_remaps,
            parameters=[LaunchConfiguration('nvblox_params_file'),
                        {'mapping_type': LaunchConfiguration('mapping_type')}],
            )
    
    # Nvblox node container
    # We use multithreaded container
    nvblox_container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[nvblox_node],
        output='screen')

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
                              mapping_type_arg,
                              nvblox_param_dir_arg,
                              sim_recorder_node,
                              nvblox_container,
                              cpu_usage_node,
                              gpu_usage_node
                              ])
