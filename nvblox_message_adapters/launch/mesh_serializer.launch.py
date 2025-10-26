# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from launch import Action, LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    actions: list[Action] = []

    actions.append(
        Node(
            package="nvblox_message_adapters",
            executable="nvblox_mesh_layer_adapter",
            name="nvblox_mesh_layer_adapter",
            output="screen",
            respawn=False,
            arguments=["--ros-args", "--log-level", "info"],
            remappings=[('mesh', '/nvblox_node/mesh')],
        ))

    return LaunchDescription(actions)
