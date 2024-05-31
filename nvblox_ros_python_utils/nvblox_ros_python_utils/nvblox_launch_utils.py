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

import enum


class EnumMeta(enum.EnumMeta):
    """Enum metaclass to add meaningful error messages for KeyErrors and AttributeErrors."""

    def __getitem__(self, name):
        try:
            return super().__getitem__(name)
        except KeyError as error:
            raise KeyError(f'The key {name} is not part of the {self.__name__} enum '
                           f'(valid options are {self.names()}). KeyError: {error}')

    def __getattr__(self, name):
        try:
            return super().__getattr__(name)
        except AttributeError as error:
            raise AttributeError(f'The attribute {name} is not part of the {self.__name__} enum. '
                                 f'AttributeError: {error}')


class NvbloxEnum(enum.Enum, metaclass=EnumMeta):
    """Base class for nvblox enums defining common functions."""

    @classmethod
    def names(cls):
        return [mode.name for mode in cls]

    def __str__(self):
        return self.name


class NvbloxMode(NvbloxEnum):
    """Enum defining the mode nvblox should be run in."""

    static = 1
    people = 2
    dynamic = 3


class NvbloxCamera(NvbloxEnum):
    """Enum defining the camera nvblox should be run with."""

    realsense = 1
    isaac_sim = 2
    zed = 3


class NvbloxPeopleSegmentation(NvbloxEnum):
    """Enum defining type of people semantic segmentation nvblox should use for people mode."""

    peoplesemsegnet_vanilla = 1
    peoplesemsegnet_shuffleseg = 2
    ground_truth = 3
