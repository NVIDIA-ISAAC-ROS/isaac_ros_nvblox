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
from launch_ros.actions import LoadComposableNodes, Node, SetParameter, SetRemap
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    output_odom_frame_name_arg = DeclareLaunchArgument(
        'output_odom_frame_name', default_value='odom',
        description='The name of the VSLAM output frame')
    run_odometry_flattening_arg = DeclareLaunchArgument(
        'run_odometry_flattening', default_value='False',
        description='Whether to flatten the output_odom_frame to 2D (camera only moving on XY-plane).')

    # Option to attach the nodes to a shared component container for speed ups through intra process communication.
    # Make sure to set the 'component_container_name' to the name of the component container you want to attach to.
    attach_to_shared_component_container_arg = LaunchConfiguration('attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration('component_container_name', default='vslam_container')

    # If we do not attach to a shared component container we have to create our own container.
    vslam_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=UnlessCondition(attach_to_shared_component_container_arg)
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[
            # Vslam node
            ComposableNode(
                name='visual_slam_node',
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode'),

            # Odom flattener node
            ComposableNode(
                name='odometry_flattener_node',
                package='odometry_flattener',
                plugin='nvblox::OdometryFlattenerNode',
                condition=IfCondition(LaunchConfiguration('run_odometry_flattening'))
            )])

    # Conditionals for setup
    setup_for_realsense = IfCondition(
        LaunchConfiguration('setup_for_realsense', default='False'))
    setup_for_isaac_sim = IfCondition(
        LaunchConfiguration('setup_for_isaac_sim', default='False'))

    # Frame name for connecting vslam to flattener
    odometry_frame_before_flattening_name = 'odom_before_flattening'

    group_action = GroupAction([
        ##########################################
        ######### VISUAL SLAM NODE SETUP #########
        ##########################################

        # Set general parameters
        SetParameter(name='enable_debug_mode', value=False),
        SetParameter(name='debug_dump_path', value='/tmp/cuvslam'),
        SetParameter(name='enable_slam_visualization', value=True),
        SetParameter(name='enable_observations_view', value=True),
        SetParameter(name='enable_landmarks_view', value=True),
        SetParameter(name='map_frame', value='map'),
        SetParameter(name='enable_localization_n_mapping', value=False),
        SetParameter(name='publish_odom_to_base_tf', value=True),
        SetParameter(name='publish_map_to_odom_tf', value=False),
        SetParameter(name='invert_odom_to_base_tf', value=True),
        # If the odometry flattener is running, the vslam output odom_frame
        # can not be set to the output_odom_frame.
        # In that case, the vslam output odom_frame is first flattened by the
        # odometry flattener node and then published as the output_odom_frame.
        SetParameter(name='odom_frame', value=LaunchConfiguration('output_odom_frame_name'),
                     condition=UnlessCondition(LaunchConfiguration('run_odometry_flattening'))),
        SetParameter(name='odom_frame', value=odometry_frame_before_flattening_name,
                     condition=IfCondition(LaunchConfiguration('run_odometry_flattening'))),

        # Parameters for Isaac Sim
        SetParameter(name='use_sim_time', value=True,
                     condition=setup_for_isaac_sim),
        SetParameter(name='denoise_input_images', value=True,
                     condition=setup_for_isaac_sim),
        SetParameter(name='rectified_images', value=False,
                     condition=setup_for_isaac_sim),
        SetParameter(name='base_frame', value='base_link',
                     condition=setup_for_isaac_sim),
        SetParameter(name='input_base_frame', value='base_link',
                     condition=setup_for_isaac_sim),
        SetParameter(name='input_left_camera_frame', value='camera_left_ROS_frame',
                     condition=setup_for_isaac_sim),
        SetParameter(name='input_right_camera_frame', value='camera_right_ROS_frame',
                     condition=setup_for_isaac_sim),

        # Parameters for Realsense
        SetParameter(name='enable_rectified_pose', value=True,
                     condition=setup_for_realsense),
        SetParameter(name='denoise_input_images', value=False,
                     condition=setup_for_realsense),
        SetParameter(name='rectified_images', value=True,
                     condition=setup_for_realsense),
        SetParameter(name='base_frame', value='camera_link',
                     condition=setup_for_realsense),

        # Remappings for Isaac Sim
        SetRemap(src=['/stereo_camera/left/camera_info'],
                 dst=['/front/stereo_camera/left/camera_info'],
                 condition=setup_for_isaac_sim),
        SetRemap(src=['/stereo_camera/right/camera_info'],
                 dst=['/front/stereo_camera/right/camera_info'],
                 condition=setup_for_isaac_sim),
        SetRemap(src=['/stereo_camera/left/image'],
                 dst=['/front/stereo_camera/left/rgb'],
                 condition=setup_for_isaac_sim),
        SetRemap(src=['/stereo_camera/right/image'],
                 dst=['/front/stereo_camera/right/rgb'],
                 condition=setup_for_isaac_sim),

        # Remappings for Realsense
        SetRemap(src=['/stereo_camera/left/camera_info'],
                 dst=['/camera/infra1/camera_info'],
                 condition=setup_for_realsense),
        SetRemap(src=['/stereo_camera/right/camera_info'],
                 dst=['/camera/infra2/camera_info'],
                 condition=setup_for_realsense),
        SetRemap(src=['/stereo_camera/left/image'],
                 dst=['/camera/realsense_splitter_node/output/infra_1'],
                 condition=setup_for_realsense),
        SetRemap(src=['/stereo_camera/right/image'],
                 dst=['/camera/realsense_splitter_node/output/infra_2'],
                 condition=setup_for_realsense),

        #################################################
        ######### ODOMETRY FLATTENER NODE SETUP #########
        #################################################

        # Set odometry flattening parameters
        SetParameter(name='input_parent_frame_id', value=odometry_frame_before_flattening_name),
        SetParameter(name='input_child_frame_id', value='base_link',
                     condition=setup_for_isaac_sim),
        SetParameter(name='input_child_frame_id', value='camera_link',
                     condition=setup_for_realsense),
        SetParameter(name='output_parent_frame_id', value=LaunchConfiguration('output_odom_frame_name')),
        SetParameter(name='output_child_frame_id', value='base_link',
                     condition=setup_for_isaac_sim),
        SetParameter(name='output_child_frame_id', value='camera_link',
                     condition=setup_for_realsense),
        SetParameter(name='invert_output_transform', value=True),

        ########################################
        ######### ADD COMPOSABLE NODES #########
        ########################################

        load_composable_nodes
    ])

    return LaunchDescription([output_odom_frame_name_arg,
                              run_odometry_flattening_arg,
                              vslam_container,
                              group_action])
