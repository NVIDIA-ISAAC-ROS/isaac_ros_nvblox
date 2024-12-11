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

from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME, \
    PEOPLENET_INPUT_IMAGE_WIDTH, PEOPLENET_INPUT_IMAGE_HEIGHT


def create_detection_pipeline(args: lu.ArgumentContainer,
                              namespace: str,
                              input_topic: str) -> List[Action]:

    # People bbox detection based on Detectnet_v2
    # ONNX masks pixels within bounding boxes with area & confidence filtering
    # Ops: Image2Tensor + TRT Node + UNet Decoder
    #    - Resolution:  network_image_resolution

    # TODO(xyao): add resize if input res != network res
    people_preprocessing_node = ComposableNode(
        name='image_to_tensor_node',
        package='isaac_ros_tensor_proc',
        plugin='nvidia::isaac_ros::dnn_inference::ImageToTensorNode',
        namespace=namespace,
        parameters=[{
            # model is trained on tensor in range of (0,1), requires pixel * 1. / 255.
            'scale': True,
            # First tensor belongs to image
            'tensor_name': args.input_tensor_names[0],
            'input_qos': 'SENSOR_DATA',
        }],
        remappings=[
            ('image', input_topic),
            ('tensor', 'detection/tensor_input'),
        ]
    )

    people_tensor_rt_node = ComposableNode(
        name='people_trt_node',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        namespace=namespace,
        parameters=[{
            'engine_file_path': args.engine_file_path,
            'output_binding_names': args.output_binding_names,
            'output_tensor_names': args.output_tensor_names,
            'input_tensor_names': args.input_tensor_names,
            'input_binding_names': args.input_binding_names,
            'force_engine_update': args.force_engine_update,
            'verbose': args.verbose,
        }],
        remappings=[
            ('tensor_pub', 'detection/tensor_input'),
            ('tensor_sub', 'detection/tensor_output')]
    )

    people_decoder_node = ComposableNode(
        name='people_decoder_node',
        package='isaac_ros_unet',
        plugin='nvidia::isaac_ros::unet::UNetDecoderNode',
        namespace=namespace,
        parameters=[{
            'network_output_type': args.network_output_type,
            'color_segmentation_mask_encoding': args.color_segmentation_mask_encoding,
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
            ('tensor_sub', 'detection/tensor_output'),
            ('unet/raw_segmentation_mask', 'detection/people_mask')])

    if args.one_container_per_camera:
        detection_node = ComposableNodeContainer(
            name=args.container_name + '_people_detection',
            package='rclcpp_components',
            namespace=namespace,
            executable='component_container_mt',
            arguments=['--ros-args', '--log-level', args.log_level],
            composable_node_descriptions=[
                people_preprocessing_node, people_tensor_rt_node, people_decoder_node
            ],
        )
    else:
        detection_node = lu.load_composable_nodes(
            args.container_name,
            [
                people_preprocessing_node, people_tensor_rt_node, people_decoder_node
            ],
        )
    return detection_node


def add_detection(args: lu.ArgumentContainer) -> List[Action]:
    # For each camera input, launch a detection node.
    # It works for both unsync and HW-sync cameras.
    assert len(args.namespace_list) == len(args.input_topic_list), \
        "Number of namespace must match number of input topic list!"
    assert len(args.input_topic_list) > 0, \
        "At least one input topic must be provided to people detection!"
    assert args.num_cameras > 0, \
        "At least one camera must be enabled to people detection!"
    assert args.num_cameras <= len(args.input_topic_list), \
        "Number of input topics must not be less than number of cameras!"
    actions = []
    if args.run_standalone and not args.one_container_per_camera:
        actions.append(lu.component_container(args.container_name))
    for i in range(args.num_cameras):
        actions.append(
            create_detection_pipeline(
                args, args.namespace_list[i], args.input_topic_list[i]))
    return actions


def generate_launch_description() -> LaunchDescription:
    '''
    Launch the resize node, DNN Image preprocessing node, TRT node and UNet decoder node
    for each camera.
    '''
    args = lu.ArgumentContainer()
    args.add_arg('num_cameras', 1,
                 description='Number of cameras requiring people detection pipeline')
    args.add_arg('namespace_list', '["camera0"]',
                 description='List of namespaces for each detection inference pipeline')
    args.add_arg(
        'input_topic_list',
        '["camera0/color/image_raw"]',
        description='List of camera image input topics for each detection inference pipeline')

    # UNet decoder mask parameters
    # mask will be the same resolution as input image
    args.add_arg('network_image_width', PEOPLENET_INPUT_IMAGE_WIDTH,
                 description='Number of columns for network input tensor image')
    args.add_arg('network_image_height', PEOPLENET_INPUT_IMAGE_HEIGHT,
                 description='Number of rows for network input tensor image')
    # TRT Node parameters
    args.add_arg('verbose', 'False',
                 description='TensorRT verbosely log if True')
    args.add_arg('force_engine_update', 'False',
                 description='TensorRT update the TensorRT engine file if True')
    args.add_arg('engine_file_path',
                 lu.get_isaac_ros_ws_path() +
                 '/isaac_ros_assets/models/peoplenet/rsu_rs_480_640_mask/1/model.plan',
                 description='Full path to detection model TRT engine')
    args.add_arg('input_binding_names', '["preprocess/input_1:0"]',
                 description='List of TRT input tensor binding names')
    args.add_arg('input_tensor_names', '["input_tensor"]',
                 description='List of TRT input tensor names')
    args.add_arg('input_tensor_formats', '["nitros_tensor_list_nchw_rgb_f32"]',
                 description='List of TRT input tensor nitros type formats')
    args.add_arg('output_tensor_names', '["people_mask"]',
                 description='List of TRT output tensor names')
    args.add_arg('output_binding_names', '["postprocess/people_mask"]',
                 description='List of TRT output tensor binding names')
    args.add_arg('output_tensor_formats', '["nitros_tensor_list_nhwc_rgb_f32"]',
                 description='List of TRT output tensor nitros type formats')

    # Additional arguments
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME,
                 description='Name of container where detection nodes are')
    args.add_arg('run_standalone', 'False',
                 description='Run in a standalone container if True')
    # To avoid fps drops in rosbag replay uncompressed sensor topics, e.g. multi-rs
    args.add_arg('one_container_per_camera', 'True',
                 description='Run per-camera based detection nodes in separate containers if True')
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)

    # Unet Decoder
    args.add_arg('network_output_type', 'sigmoid')
    args.add_arg('color_segmentation_mask_encoding', 'rgb8')
    args.add_opaque_function(add_detection)
    return LaunchDescription(args.get_launch_actions())
