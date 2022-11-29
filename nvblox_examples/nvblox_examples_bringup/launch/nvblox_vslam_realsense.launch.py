# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():

    # RealSense
    realsense_config_file_path = os.path.join(
        get_package_share_directory('nvblox_examples_bringup'),
        'config', 'realsense.yaml'
    )  

    realsense_node = ComposableNode(
        namespace="camera",
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[realsense_config_file_path],
    )

    realsense_splitter_node = ComposableNode(
        namespace="camera",
        name='realsense_splitter_node',
        package='realsense_splitter',
        plugin='nvblox::RealsenseSplitterNode',
        parameters=[{
                    'input_qos': 'SENSOR_DATA',
                    'output_qos': 'SENSOR_DATA'
        }],
        remappings=[('input/infra_1', '/camera/infra1/image_rect_raw'),
                    ('input/infra_1_metadata', '/camera/infra1/metadata'),
                    ('input/infra_2', '/camera/infra2/image_rect_raw'),
                    ('input/infra_2_metadata', '/camera/infra2/metadata'),
                    ('input/depth', '/camera/depth/image_rect_raw'),
                    ('input/depth_metadata', '/camera/depth/metadata'),
                    ('input/pointcloud', '/camera/depth/color/points'),
                    ('input/pointcloud_metadata', '/camera/depth/metadata'),
        ]
    )

    realsense_container = ComposableNodeContainer(
        name='realsense_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            realsense_node,
            realsense_splitter_node
        ],
        output='screen'
    )

    # VSLAM
    visual_slam_node = Node(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        parameters=[{
                    'enable_rectified_pose': True,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/vslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'input_left_camera_frame': 'camera_infra1_frame',
                    'input_right_camera_frame': 'camera_infra2_frame',
                    'enable_localization_n_mapping': True,
                    'publish_odom_to_base_tf': True,
                    'publish_map_to_odom_tf': True,
                    'image_qos': 'SENSOR_DATA'
        }],
        remappings=[('stereo_camera/left/image', '/camera/realsense_splitter_node/output/infra_1'),
                    ('stereo_camera/left/camera_info', '/camera/infra1/camera_info'),
                    ('stereo_camera/right/image', '/camera/realsense_splitter_node/output/infra_2'),
                    ('stereo_camera/right/camera_info', '/camera/infra2/camera_info')]
    )

    base_link_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0', '0', '0', '0', '1',
            'base_link', 'camera_link']
    )

    # Nvblox
    nvblox_config = DeclareLaunchArgument(
        'nvblox_config', default_value=os.path.join(
            get_package_share_directory(
                'nvblox_examples_bringup'), 'config', 'nvblox.yaml'
        )
    )

    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        parameters=[LaunchConfiguration('nvblox_config')],
        output='screen',
        remappings=[
            ("depth/camera_info", "/camera/depth/camera_info"),
            ("depth/image", "/camera/realsense_splitter_node/output/depth"),
            ("color/camera_info", "/camera/color/camera_info"),
            ("color/image", "/camera/color/image_raw")
        ]
    )

    # RVIZ
    rviz_config_path = os.path.join(get_package_share_directory(
        'nvblox_examples_bringup'), 'config', 'nvblox_vslam_realsense.rviz')

    print(rviz_config_path)

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')

    return LaunchDescription([
        realsense_container,
        nvblox_config,
        visual_slam_node,
        nvblox_node,
        base_link_tf_node,
        rviz
    ])
