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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import UnlessCondition


def generate_launch_description():
    
    """Launch the DNN Image encoder, Triton node and UNet decoder node, with the padding and depadding nodes"""
    launch_args = [
        DeclareLaunchArgument(
            'network_image_width',
            default_value='960',
            description='The input image width that the network expects'),
        DeclareLaunchArgument(
            'network_image_height',
            default_value='544',
            description='The input image height that the network expects'),
        DeclareLaunchArgument(
            'encoder_image_mean',
            default_value='[0.5, 0.5, 0.5]',
            description='The mean for image normalization'),
        DeclareLaunchArgument(
            'encoder_image_stddev',
            default_value='[0.5, 0.5, 0.5]',
            description='The standard deviation for image normalization'),
        DeclareLaunchArgument(
            'model_name',
            default_value='',
            description='The name of the model'),
        DeclareLaunchArgument(
            'model_repository_paths',
            default_value='[""]',
            description='The absolute path to the repository of models'),
        DeclareLaunchArgument(
            'max_batch_size',
            default_value='0',
            description='The maximum allowed batch size of the model'),
        DeclareLaunchArgument(
            'input_tensor_names',
            default_value='["input_tensor"]',
            description='A list of tensor names to bound to the specified input binding names'),
        DeclareLaunchArgument(
            'input_binding_names',
            default_value='["input_1"]',
            description='A list of input tensor binding names (specified by model)'),
        DeclareLaunchArgument(
            'input_tensor_formats',
            default_value='["nitros_tensor_list_nchw_rgb_f32"]',
            description='The nitros format of the input tensors'),
        DeclareLaunchArgument(
            'output_tensor_names',
            default_value='["output_tensor"]',
            description='A list of tensor names to bound to the specified output binding names'),
        DeclareLaunchArgument(
            'output_binding_names',
            default_value='["softmax_1"]',
            description='A  list of output tensor binding names (specified by model)'),
        DeclareLaunchArgument(
            'output_tensor_formats',
            default_value='["nitros_tensor_list_nhwc_rgb_f32"]',
            description='The nitros format of the output tensors'),
        DeclareLaunchArgument(
            'network_output_type',
            default_value='argmax',
            description='The output type that the network provides (softmax, sigmoid or argmax)'),
        DeclareLaunchArgument(
            'color_segmentation_mask_encoding',
            default_value='rgb8',
            description='The image encoding of the colored segmentation mask (rgb8 or bgr8)'),
        DeclareLaunchArgument(
            'mask_width',
            default_value='960',
            description='The width of the segmentation mask'),
        DeclareLaunchArgument(
            'mask_height',
            default_value='544',
            description='The height of the segmentation mask'),
    ]
    
    # DNN Image Encoder parameters
    network_image_width = LaunchConfiguration('network_image_width')
    network_image_height = LaunchConfiguration('network_image_height')
    encoder_image_mean = LaunchConfiguration('encoder_image_mean')
    encoder_image_stddev = LaunchConfiguration('encoder_image_stddev')
    
    # Triton parameters
    model_name = LaunchConfiguration('model_name')
    model_repository_paths = LaunchConfiguration('model_repository_paths')
    max_batch_size = LaunchConfiguration('max_batch_size')
    input_tensor_names = LaunchConfiguration('input_tensor_names')
    input_binding_names = LaunchConfiguration('input_binding_names')
    input_tensor_formats = LaunchConfiguration('input_tensor_formats')
    output_tensor_names = LaunchConfiguration('output_tensor_names')
    output_binding_names = LaunchConfiguration('output_binding_names')
    output_tensor_formats = LaunchConfiguration('output_tensor_formats')

    # U-Net Decoder parameters
    network_output_type = LaunchConfiguration('network_output_type')
    color_segmentation_mask_encoding = LaunchConfiguration(
        'color_segmentation_mask_encoding')
    mask_width = LaunchConfiguration('mask_width')
    mask_height = LaunchConfiguration('mask_height')

    # Padding and De-padding parameters
    padding_input_topic = LaunchConfiguration('padding_input_topic', default='/camera/color/image_raw')
    
    # Option to attach the nodes to a shared component container for speed ups through intra process communication.
    # Make sure to set the 'component_container_name' to the name of the component container you want to attach to.
    attach_to_shared_component_container_arg = LaunchConfiguration('attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration('component_container_name', default='segmentation_container')

    # If we do not attach to a shared component container we have to create our own container.
    segmentation_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=UnlessCondition(attach_to_shared_component_container_arg)
    )

    # Semantic segmentation
    load_composable_nodes = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[
            ComposableNode(
                name='dnn_image_encoder',
                package='isaac_ros_dnn_image_encoder',
                plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
                parameters=[{
                    'input_image_width': network_image_width,
                    'input_image_height': network_image_height,
                    'network_image_width': network_image_width,
                    'network_image_height': network_image_height,
                    'image_mean': encoder_image_mean,
                    'image_stddev': encoder_image_stddev,
                }],
                remappings=[('encoded_tensor', 'tensor_pub'), ('image', '/camera/color/image_raw_padded')]
            ),
            # Triton Node
            ComposableNode(
                name='triton_node',
                package='isaac_ros_triton',
                plugin='nvidia::isaac_ros::dnn_inference::TritonNode',
                parameters=[{
                    'model_name': model_name,
                    'model_repository_paths': model_repository_paths,
                    'max_batch_size': max_batch_size,
                    'input_tensor_names': input_tensor_names,
                    'input_binding_names': input_binding_names,
                    'input_tensor_formats': input_tensor_formats,
                    'output_tensor_names': output_tensor_names,
                    'output_binding_names': output_binding_names,
                    'output_tensor_formats': output_tensor_formats,
                }]),
            # Unet Decoder Node
            ComposableNode(
                name='unet_decoder_node',
                package='isaac_ros_unet',
                plugin='nvidia::isaac_ros::unet::UNetDecoderNode',
                parameters=[{
                    'network_output_type': network_output_type,
                    'color_segmentation_mask_encoding': color_segmentation_mask_encoding,
                    'mask_width': mask_width,
                    'mask_height': mask_height,
                    'color_palette': [0x556B2F, 0x800000, 0x008080, 0x000080, 0x9ACD32, 0xFF0000, 0xFF8C00,
                                      0xFFD700, 0x00FF00, 0xBA55D3, 0x00FA9A, 0x00FFFF, 0x0000FF, 0xF08080,
                                      0xFF00FF, 0x1E90FF, 0xDDA0DD, 0xFF1493, 0x87CEFA, 0xFFDEAD],
                }]),
            # Encoder - Padding
            ComposableNode(
            name='image_padding_node',
            package='nvblox_image_padding',
            plugin='nvblox::ImagePaddingCroppingNode',
            parameters=[{'image_qos': 'SYSTEM_DEFAULT',
                         'desired_height': 544,
                         'desired_width': 960
                         }],
            remappings=[('~/image_in', padding_input_topic),
                        ('~/image_out', '/camera/color/image_raw_padded')]),
            # Decoder - Depadding
            ComposableNode(
            name='image_cropping_node',
            package='nvblox_image_padding',
            plugin='nvblox::ImagePaddingCroppingNode',
            parameters=[{'image_qos': 'SENSOR_DATA',
                         'desired_height': 480,
                         'desired_width': 640
                         }],
            remappings=[('~/image_in', '/unet/raw_segmentation_mask'),
                        ('~/image_out', '/unet/raw_segmentation_mask_depadded')])])

    final_launch_description = launch_args + [segmentation_container, load_composable_nodes]
    return LaunchDescription(final_launch_description)
