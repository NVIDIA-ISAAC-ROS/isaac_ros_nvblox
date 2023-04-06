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

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    output_odom_frame_name_arg = DeclareLaunchArgument(
        'output_odom_frame_name', default_value='odom',
        description='The name of the VSLAM output frame')

    # VSLAM node
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'enable_rectified_pose': True,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/elbrus',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': LaunchConfiguration('output_odom_frame_name'),
                    'base_frame': 'camera_link',
                    'enable_localization_n_mapping': False,
                    'publish_odom_to_base_tf': True,
                    'publish_map_to_odom_tf': False,
                    'invert_odom_to_base_tf': True,
                    }],
        remappings=[('stereo_camera/left/image',
                     '/camera/realsense_splitter_node/output/infra_1'),
                    ('stereo_camera/left/camera_info',
                     '/camera/infra1/camera_info'),
                    ('stereo_camera/right/image',
                     '/camera/realsense_splitter_node/output/infra_2'),
                    ('stereo_camera/right/camera_info',
                    '/camera/infra2/camera_info')])

    # VSLAM container
    vslam_container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen')

    return LaunchDescription([
        output_odom_frame_name_arg,
        vslam_container])
