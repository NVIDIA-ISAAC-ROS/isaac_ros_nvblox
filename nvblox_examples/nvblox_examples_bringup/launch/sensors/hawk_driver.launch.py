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

from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition


def generate_launch_description():

    # NITROS nodes are required to start together to allow format negotiation
    # to take place before all the NITROS nodes are ready to start their underlying GXF graphs.
    # NOTE(remos): For big pipelines the negotiation time
    # can be increased with the type_negotiation_duration_s parameter.
    type_negotiation_duration_s_arg = LaunchConfiguration(
        'type_negotiation_duration_s', default=1)

    # ID of the camera to use
    hawk_module_id_arg = LaunchConfiguration('hawk_module_id', default=5)

    # Option to attach the nodes to a shared component container for speed ups through intra process communication.
    # Make sure to set the 'component_container_name' to the name of the component container you want to attach to.
    attach_to_shared_component_container_arg = LaunchConfiguration(
        'attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration(
        'component_container_name', default='hawk_driver_container')

    # If we do not attach to a shared component container we have to create our own container.
    hawk_driver_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=UnlessCondition(attach_to_shared_component_container_arg))

    # Transform between IMU and camera
    # NOTE(remos): Taken from cuVSLAM hawk launch files.
    argus_imu_camera_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0947', '0.0061', '0.0', '0.0', '0.70710678', '0.0',
            '0.70710678', 'camera', 'bmi088_frame'
        ])

    load_composable_nodes = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[
            # Timestamp driver
            ComposableNode(
                package='isaac_ros_correlated_timestamp_driver',
                plugin=
                'nvidia::isaac_ros::correlated_timestamp_driver::CorrelatedTimestampDriverNode',
                name='correlated_timestamp_driver',
                namespace='hawk',
                parameters=[{
                    'use_time_since_epoch':
                    False,
                    'nvpps_dev_name':
                    '/dev/nvpps0',
                    'type_negotiation_duration_s':
                    type_negotiation_duration_s_arg,
                }]),
            # Hawk driver
            ComposableNode(
                name='hawk_camera_driver',
                package='isaac_ros_hawk',
                plugin='nvidia::isaac_ros::hawk::HawkNode',
                namespace='hawk',
                # NOTE(remos): module_id/fsync_type parameters are set for carter v2.4
                remappings=[
                    ('left/image_raw', 'left/image_raw'),
                    ('right/image_raw', 'right/image_raw'),
                    ('left/camerainfo', 'left/camera_info'),
                    ('right/camerainfo', 'right/camera_info'),
                    ('left/image_raw', 'left/image_raw/nitros'),
                    ('right/image_raw', 'right/image_raw/nitros'),
                    ('left/camerainfo', 'left/camera_info/nitros'),
                    ('right/camerainfo', 'right/camera_info/nitros'),
                ],
                parameters=[{
                    'module_id':
                    hawk_module_id_arg,
                    'fsync_type':
                    1,
                    'camera_link_frame_name':
                    'camera_link',
                    'left_optical_frame_name':
                    'camera_link_left_optical',
                    'right_optical_frame_name':
                    'camera_link_right_optical',
                    'type_negotiation_duration_s':
                    type_negotiation_duration_s_arg,
                }]),

            # IMU driver
            # NOTE(remos): We are not using the IMU at the moment.
            # We still add it here as recorded IMU data can help with cuVSLAM debugging.
            ComposableNode(name='bmi088_imu_driver',
                           package='isaac_ros_imu_bmi088',
                           plugin='nvidia::isaac_ros::imu_bmi088::Bmi088Node',
                           namespace='hawk',
                           parameters=[{
                               'imu_frequency':
                               200,
                               'accel_index':
                               0,
                               'gyro_index':
                               1,
                               'iio_buf_size':
                               64,
                               'type_negotiation_duration_s':
                               type_negotiation_duration_s_arg,
                           }])
        ])

    return LaunchDescription([
        hawk_driver_container, load_composable_nodes,
        argus_imu_camera_static_transform
    ])
