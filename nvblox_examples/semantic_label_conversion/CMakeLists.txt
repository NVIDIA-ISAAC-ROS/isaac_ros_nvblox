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

cmake_minimum_required(VERSION 3.5)
project(semantic_label_conversion)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

# find dependencies
find_package(ament_cmake REQUIRED)

# Installing script so their runable
install(PROGRAMS
  semantic_label_conversion/semantic_label_converter.py
  semantic_label_conversion/semantic_label_stamper.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY params launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
