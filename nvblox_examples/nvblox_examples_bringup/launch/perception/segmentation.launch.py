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

import os
from typing import List

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxPeopleSegmentation
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME, \
    SEMSEGNET_INPUT_IMAGE_WIDTH, SEMSEGNET_INPUT_IMAGE_HEIGHT


def add_segmentation(args: lu.ArgumentContainer) -> List[Action]:
    people_segmentation = NvbloxPeopleSegmentation[args.people_segmentation]
    if people_segmentation is NvbloxPeopleSegmentation.peoplesemsegnet_vanilla:
        model_name = 'deployable_quantized_vanilla_unet_v2.0'
        input_binding_names = ['input_1:0']
    elif people_segmentation is NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg:
        model_name = 'deployable_shuffleseg_unet_amr_v1.0'
        input_binding_names = ['input_2:0']
    else:
        raise Exception(f'People segmentation mode {people_segmentation} not implemented.')


    # Semantic segmentation
    # 1) Input Padding / Cropping:
    #    - Input Resolution:  input_image_resolution
    #    - Output Resolution: network_image_resolution
    # 2) Unet Encoder + Triton Node + Unet Decoder
    #    - Resolution:        network_image_resolution
    # 1) Output Padding / Cropping:
    #    - Input Resolution:  network_image_resolution
    #    - Output Resolution: input_image_resolution

    resize_node = ComposableNode(
        name='segmentation_resize_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'output_width': args.network_image_width,
            'output_height': args.network_image_height,
            'keep_aspect_ratio': False,
            'input_qos': 'SENSOR_DATA',
        }],
        remappings=[
            ('image', args.input_topic),
            ('camera_info', args.input_camera_info_topic),
            ('resize/image', '/segmentation/image_resized'),
            ('resize/camera_info', '/segmentation/camera_info_resized'),
        ]
    )

    unet_encoder_node = ComposableNode(
        name='dnn_image_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        parameters=[{
            # Do not resize (we do padding / cropping):
            # input resolution = network resolution
            'input_image_width': args.network_image_width,
            'input_image_height': args.network_image_height,
            'network_image_width': args.network_image_width,
            'network_image_height': args.network_image_height,
            'image_mean': args.encoder_image_mean,
            'image_stddev': args.encoder_image_stddev,
        }],
        remappings=[
            ('encoded_tensor', 'tensor_pub'),
            ('image', '/segmentation/image_resized')])

    triton_node = ComposableNode(
        name='triton_node',
        package='isaac_ros_triton',
        plugin='nvidia::isaac_ros::dnn_inference::TritonNode',
        parameters=[{
            'model_name': model_name,
            'model_repository_paths': args.model_repository_paths,
            'max_batch_size': args.max_batch_size,
            'input_tensor_names': args.input_tensor_names,
            'input_binding_names': input_binding_names,
            'input_tensor_formats': args.input_tensor_formats,
            'output_tensor_names': args.output_tensor_names,
            'output_binding_names': args.output_binding_names,
            'output_tensor_formats': args.output_tensor_formats,
        }])

    unet_decoder_node = ComposableNode(
        name='unet_decoder_node',
        package='isaac_ros_unet',
        plugin='nvidia::isaac_ros::unet::UNetDecoderNode',
        parameters=[{
            'network_output_type': args.network_output_type,
            'color_segmentation_mask_encoding': args.color_segmentation_mask_encoding,
            # Do not resize (we do padding / cropping):
            # network resolution = output mask resolution
            'mask_width': args.network_image_width,
            'mask_height': args.network_image_height,
            'color_palette': [
                0x556B2F, 0x800000, 0x008080, 0x000080, 0x9ACD32, 0xFF0000, 0xFF8C00, 0xFFD700,
                0x00FF00, 0xBA55D3, 0x00FA9A, 0x00FFFF, 0x0000FF, 0xF08080, 0xFF00FF, 0x1E90FF,
                0xDDA0DD, 0xFF1493, 0x87CEFA, 0xFFDEAD
            ],
        }])

    actions = []
    if args.run_standalone:
        actions.append(lu.component_container(args.container_name))
    actions.append(
        lu.load_composable_nodes(
            args.container_name,
            [
                unet_encoder_node, triton_node, unet_decoder_node, resize_node,
            ],
        ))

    return actions


def generate_launch_description() -> LaunchDescription:
    """Launch the DNN Image encoder, Triton node and UNet decoder node, with the padding and depadding nodes"""
    args = lu.ArgumentContainer()
    args.add_arg('people_segmentation', NvbloxPeopleSegmentation.peoplesemsegnet_vanilla)
    # Inputs
    args.add_arg('input_topic')
    args.add_arg('input_camera_info_topic')
    # DNN Image Encoder parameters
    args.add_arg('network_image_width', SEMSEGNET_INPUT_IMAGE_WIDTH)
    args.add_arg('network_image_height', SEMSEGNET_INPUT_IMAGE_HEIGHT)
    args.add_arg('encoder_image_mean', '[0.5, 0.5, 0.5]')
    args.add_arg('encoder_image_stddev', '[0.5, 0.5, 0.5]')
    # Triton parameters
    args.add_arg('model_repository_paths',
                 '["' + lu.get_isaac_ros_ws_path() + '/isaac_ros_assets/models/peoplesemsegnet"]')
    args.add_arg('max_batch_size', '0')
    args.add_arg('input_tensor_names', '["input_tensor"]')
    args.add_arg('input_tensor_formats', '["nitros_tensor_list_nchw_rgb_f32"]')
    args.add_arg('output_tensor_names', '["output_tensor"]')
    args.add_arg('output_binding_names', '["argmax_1"]')
    args.add_arg('output_tensor_formats', '["nitros_tensor_list_nhwc_rgb_f32"]')
    # U-Net Decoder parameters
    args.add_arg('network_output_type', 'argmax')
    args.add_arg('color_segmentation_mask_encoding', 'rgb8')
    # Additional arguments
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')

    args.add_opaque_function(add_segmentation)
    return LaunchDescription(args.get_launch_actions())
