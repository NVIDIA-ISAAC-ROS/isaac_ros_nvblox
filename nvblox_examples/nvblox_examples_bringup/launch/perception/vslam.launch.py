# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from typing import List

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxCamera
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def add_vslam(args: lu.ArgumentContainer) -> List[Action]:
    actions = []

    camera = NvbloxCamera[args.camera]
    # NOTE(alexmillane, 19.08.2024): At the moment in nvblox_examples we only support a single
    # camera running cuVSLAM, even in the multi-camera case: we run *nvblox* on multiple
    # cameras, but cuVSLAM on camera0 only.
    realsense_remappings = [
        ('visual_slam/camera_info_0', '/camera0/infra1/camera_info'),
        ('visual_slam/camera_info_1', '/camera0/infra2/camera_info'),
        ('visual_slam/image_0', '/camera0/realsense_splitter_node/output/infra_1'),
        ('visual_slam/image_1', '/camera0/realsense_splitter_node/output/infra_2'),
        ('visual_slam/imu', 'camera0/imu'),
    ]

    # Base frame: 
    # - camera0_link for single realsense,
    # - base_link for everything else (multi realsense)
    if camera is NvbloxCamera.realsense:
        base_frame = 'camera0_link'
    else:
        base_frame = 'base_link'

    actions.append(lu.log_info(f'Starting cuVSLAM with base_frame: {base_frame}'))

    base_parameters = {
        'num_cameras': 2,
        'min_num_images': 2,
        'enable_localization_n_mapping': False,
        'gyro_noise_density': 0.000244,
        'gyro_random_walk': 0.000019393,
        'accel_noise_density': 0.001862,
        'accel_random_walk': 0.003,
        'calibration_frequency': 200.0,
        'rig_frame': 'base_link',
        'imu_frame': 'front_stereo_camera_imu',
        'enable_slam_visualization': True,
        'enable_landmarks_view': True,
        'enable_observations_view': True,
        'path_max_size': 200,
        'verbosity': 5,
        'enable_debug_mode': False,
        'debug_dump_path': '/tmp/cuvslam',
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_frame': base_frame,
    }
    realsense_parameters = {
        'enable_rectified_pose': True,
        'enable_image_denoising': False,
        'rectified_images': True,
        'imu_frame': 'camera0_gyro_optical_frame',
        'camera_optical_frames': [
            'camera0_infra1_optical_frame',
            'camera0_infra2_optical_frame',
        ],
    }

    if camera is NvbloxCamera.realsense or NvbloxCamera.multi_realsense:
        remappings = realsense_remappings
        camera_parameters = realsense_parameters
    else:
        raise Exception(f'Camera {camera} not implemented for vslam.')

    parameters = []
    parameters.append(base_parameters)
    parameters.append(camera_parameters)
    parameters.append(
        {'enable_ground_constraint_in_odometry': args.enable_ground_constraint_in_odometry})
    parameters.append({'enable_imu_fusion': args.enable_imu_fusion})

    vslam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=remappings,
        parameters=parameters)
    actions.append(lu.load_composable_nodes(args.container_name, [vslam_node]))

    if args.run_standalone:
        actions.append(lu.component_container(args.container_name))

    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('camera')
    args.add_arg(
        'enable_ground_constraint_in_odometry',
        'False',
        description='Whether to constraint robot movement to a 2d plane (e.g. for AMRs).',
        cli=True)
    args.add_arg(
        'enable_imu_fusion',
        'False',
        description='Whether to use imu data in visual slam.',
        cli=True)
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')
    args.add_opaque_function(add_vslam)
    return LaunchDescription(args.get_launch_actions())
