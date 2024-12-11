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

from launch import Action, LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxPeopleSegmentation
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME, \
    SEMSEGNET_INPUT_IMAGE_WIDTH, SEMSEGNET_INPUT_IMAGE_HEIGHT


def create_segmentation_pipeline(args: lu.ArgumentContainer,
                                 namespace: str,
                                 input_topic: str,
                                 input_camera_info_topic: str,
                                 output_resized_image_topic: str,
                                 output_resized_camera_info_topic: str) -> List[Action]:

    people_segmentation = NvbloxPeopleSegmentation[args.people_segmentation]
    if people_segmentation is NvbloxPeopleSegmentation.peoplesemsegnet_vanilla:
        engine_file_path = args.vanilla_engine_file_path
        input_binding_names = ['input_1:0']
    elif people_segmentation is NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg:
        engine_file_path = args.shuffleseg_engine_file_path
        input_binding_names = ['input_2']
    else:
        raise Exception(f'People segmentation mode {people_segmentation} not implemented.')

    # Semantic segmentation
    # 1) Input Padding / Cropping:
    #    - Input Resolution:  input_image_resolution
    #    - Output Resolution: network_image_resolution
    # 2) Image2Tensor + Swap axis (Vanilla) / Image2Tensor(ShuffleSeg) + TRT Node + Unet Decoder
    #    - Resolution:        network_image_resolution
    # 1) Output Padding / Cropping:
    #    - Input Resolution:  network_image_resolution
    #    - Output Resolution: input_image_resolution

    resize_node = ComposableNode(
        name='segmentation_resize_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        namespace=namespace,
        parameters=[{
            'output_width': args.network_image_width,
            'output_height': args.network_image_height,
            'keep_aspect_ratio': False,
            'input_qos': 'SENSOR_DATA',
        }],
        remappings=[
            ('image', input_topic),
            ('camera_info', input_camera_info_topic),
            ('resize/image', output_resized_image_topic),
            ('resize/camera_info', output_resized_camera_info_topic),
        ]
    )

    # Only needs VideoBuffer2Tensor conversion, other pre-processing ops are in ONNX
    if people_segmentation is NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg:
        people_preprocessing_node = ComposableNode(
            name='image_to_tensor_node',
            package='isaac_ros_tensor_proc',
            plugin='nvidia::isaac_ros::dnn_inference::ImageToTensorNode',
            namespace=namespace,
            parameters=[{
                'scale': True,
                # First tensor belongs to image
                'tensor_name': args.input_tensor_names[0],
            }],
            remappings=[
                ('image', output_resized_image_topic),
                ('tensor', 'segmentation/tensor_input'),
            ]
        )
    # people_segmentation shall only be either vanilla or shuffleseg, exception shall be caught
    # in the start of thisfunction
    else:
        # DnnImageEncoderNode duplicates output twice on RealSense live
        people_preprocessing_node = ComposableNode(
            name='image_to_tensor_node',
            package='isaac_ros_tensor_proc',
            plugin='nvidia::isaac_ros::dnn_inference::ImageToTensorNode',
            namespace=namespace,
            parameters=[{
                'scale': True,
                # First tensor belongs to image
                'tensor_name': args.input_tensor_names[0],
            }],
            remappings=[
                ('image', output_resized_image_topic),
                ('tensor', 'segmentation/image_to_tensor_output'),
            ]
        )
        # NHWC -> NCHW
        people_bchw_node = ComposableNode(
            name='interleaved_to_planar_node',
            package='isaac_ros_tensor_proc',
            plugin='nvidia::isaac_ros::dnn_inference::InterleavedToPlanarNode',
            namespace=namespace,
            parameters=[
                {
                    'input_tensor_shape': [args.network_image_height, args.network_image_width, 3],
                    'num_blocks': 40,
                }
            ],
            remappings=[
                ('interleaved_tensor', 'segmentation/image_to_tensor_output'),
                ('planar_tensor', 'segmentation/tensor_input')
            ],
        )

    people_tensor_rt_node = ComposableNode(
        name='people_trt_node',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        namespace=namespace,
        parameters=[{
            'engine_file_path': engine_file_path,
            'output_binding_names': args.output_binding_names,
            'output_tensor_names': args.output_tensor_names,
            'input_tensor_names': args.input_tensor_names,
            'input_binding_names': input_binding_names,
            'force_engine_update': args.force_engine_update,
            'verbose': args.verbose,
        }],
        remappings=[
            ('tensor_pub', 'segmentation/tensor_input'),
            ('tensor_sub', 'segmentation/tensor_output')
        ]
    )

    people_decoder_node = ComposableNode(
        name='unet_decoder_node',
        package='isaac_ros_unet',
        plugin='nvidia::isaac_ros::unet::UNetDecoderNode',
        namespace=namespace,
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
        }],
        remappings=[
            ('tensor_sub', 'segmentation/tensor_output'),
            ('unet/raw_segmentation_mask', 'segmentation/people_mask')
        ]
    )
    nodes_list = [
        resize_node, people_preprocessing_node, people_tensor_rt_node, people_decoder_node
    ]
    if people_segmentation is NvbloxPeopleSegmentation.peoplesemsegnet_vanilla:
        nodes_list.append(people_bchw_node)

    if args.one_container_per_camera:
        segmentation_node = ComposableNodeContainer(
            name=args.container_name + '_people_segmentation',
            package='rclcpp_components',
            namespace=namespace,
            executable='component_container_mt',
            arguments=['--ros-args', '--log-level', args.log_level],
            composable_node_descriptions=nodes_list,
        )
    else:
        segmentation_node = lu.load_composable_nodes(args.container_name, nodes_list)

    return segmentation_node


def add_segmentation(args: lu.ArgumentContainer) -> List[Action]:
    # For each camera input, launch a segmentation node.
    # It works for both unsync and HW-sync cameras.
    assert len(args.namespace_list) == len(args.input_topic_list), \
        "Number of namespace must match number of input topic list!"
    assert len(args.input_camera_info_topic_list) == len(args.input_topic_list), \
        "Number of input camera info topic list must match number of input topic list!"
    assert len(args.output_resized_image_topic_list) == len(args.input_topic_list), \
        "Number of output resized image topic list must match number of input topic list!"
    assert len(args.output_resized_camera_info_topic_list) == len(args.input_topic_list), \
        "Number of output resized camera info topic list must match number of input topic list!"
    assert len(args.input_topic_list) > 0, \
        "At least one input topic must be provided to people segmentation!"
    assert args.num_cameras > 0, \
        "At least one camera must be enabled to people segmentation!"
    assert args.num_cameras <= len(args.input_topic_list), \
        "Number of input topics must not be less than number of cameras!"
    actions = []
    if args.run_standalone and not args.one_container_per_camera:
        actions.append(lu.component_container(args.container_name))
    for i in range(args.num_cameras):
        actions.append(
            create_segmentation_pipeline(
                args,
                namespace=args.namespace_list[i],
                input_topic=args.input_topic_list[i],
                input_camera_info_topic=args.input_camera_info_topic_list[i],
                output_resized_image_topic=args.output_resized_image_topic_list[i],
                output_resized_camera_info_topic=args.output_resized_camera_info_topic_list[i]))
    return actions


def generate_launch_description() -> LaunchDescription:
    '''
    Launch the DNN Image encoder, Triton node and UNet decoder node,
    with the padding and depadding nodes
    '''
    args = lu.ArgumentContainer()
    args.add_arg('people_segmentation',
                 NvbloxPeopleSegmentation.peoplesemsegnet_vanilla,
                 choices=[NvbloxPeopleSegmentation.peoplesemsegnet_vanilla,
                          NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg],
                 description='People Segmentaion model')
    args.add_arg('num_cameras', 1,
                 description='Number of cameras requiring people segmentation pipeline')
    args.add_arg('namespace_list', '["camera0"]',
                 description='List of namespaces for each segmentation inference pipeline')

    args.add_arg(
        'input_topic_list',
        '["camera0/color/image_raw"]',
        description='List of camera image input topics for each segmentation inference pipeline')
    args.add_arg(
        'input_camera_info_topic_list',
        '["camera0/color/camera_info"]',
        description='List of input camera info topics for each segmentation inference pipeline')
    args.add_arg(
        'output_resized_image_topic_list',
        '["camera0/segmentation/image_resized"]',
        description='List of output resized image topics for each segmentation inference pipeline')
    args.add_arg(
        'output_resized_camera_info_topic_list',
        '["camera0/segmentation/camera_info_resized"]',
        description='List of output resized camera info topics for each segmentation pipeline')

    # DNN Image preprocessing parameters
    args.add_arg('network_image_width', SEMSEGNET_INPUT_IMAGE_WIDTH,
                 description='Number of columns for network input tensor image')
    args.add_arg('network_image_height', SEMSEGNET_INPUT_IMAGE_HEIGHT,
                 description='Number of rows for network input tensor image')
    # TRT Node parameters
    args.add_arg('verbose', 'False',
                 description='TensorRT verbosely log if True')
    args.add_arg('force_engine_update', 'False',
                 description='TensorRT update the TensorRT engine file if True')
    default_base_dir = '/isaac_ros_assets/models/peoplesemsegnet'
    args.add_arg('shuffleseg_engine_file_path',
                 lu.get_isaac_ros_ws_path() +
                 f'{default_base_dir}/optimized_deployable_shuffleseg_unet_amr_v1.0/1/model.plan',
                 description='Full path to shuffleseg model TRT engine')
    args.add_arg('vanilla_engine_file_path',
                 lu.get_isaac_ros_ws_path() +
                 f'{default_base_dir}/deployable_quantized_vanilla_unet_onnx_v2.0/1/model.plan',
                 description='Full path to vanilla model TRT engine')
    args.add_arg('input_tensor_names', '["input_tensor"]',
                 description='List of TRT input tensor names')
    args.add_arg('input_tensor_formats', '["nitros_tensor_list_nchw_rgb_f32"]',
                 description='List of TRT input tensor nitros type formats')
    args.add_arg('output_tensor_names', '["output_tensor"]',
                 description='List of TRT output tensor names')
    args.add_arg('output_binding_names', '["argmax_1"]',
                 description='List of TRT output tensor binding names')
    args.add_arg('output_tensor_formats', '["nitros_tensor_list_nhwc_rgb_f32"]',
                 description='List of TRT output tensor nitros type formats')
    # U-Net Decoder parameters
    args.add_arg('network_output_type', 'argmax')
    args.add_arg('color_segmentation_mask_encoding', 'rgb8')
    # Additional arguments
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME,
                 description='Name of container where segmentation nodes are')
    args.add_arg('run_standalone', 'False',
                 description='Run in a standalone container if True')
    # To avoid fps drops in rosbag replay uncompressed sensor topics, e.g. multi-rs
    args.add_arg('one_container_per_camera', 'True',
                 description='Run per-camera based segmentation nodes '
                             'in separate containers if True')
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)

    args.add_opaque_function(add_segmentation)
    return LaunchDescription(args.get_launch_actions())
