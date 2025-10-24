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

import importlib.util
from pathlib import Path
import sys

from ament_index_python.packages import get_resource
from setuptools import setup

ISAAC_ROS_COMMON_PATH = get_resource(
    'isaac_ros_common_scripts_path',
    'isaac_ros_common'
)[0]

ISAAC_ROS_COMMON_VERSION_INFO = Path(ISAAC_ROS_COMMON_PATH) / 'isaac_ros_common-version-info.py'

spec = importlib.util.spec_from_file_location(
    'isaac_ros_common_version_info',
    ISAAC_ROS_COMMON_VERSION_INFO
)

isaac_ros_common_version_info = importlib.util.module_from_spec(spec)
sys.modules['isaac_ros_common_version_info'] = isaac_ros_common_version_info
spec.loader.exec_module(isaac_ros_common_version_info)

from isaac_ros_common_version_info import GenerateVersionInfoCommand  # noqa: E402, I100

package_name = 'nvblox_test'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    extras_require={
        'test': [
            'pytest'
        ]
    },
    zip_safe=True,
    maintainer='Isaac ROS Maintainers',
    maintainer_email='isaac-ros-maintainers@nvidia.com',
    description='Tests for nvblox top level launch files',
    license='Apache-2.0',
    cmdclass={
        'build_py': GenerateVersionInfoCommand,
    },
)
