// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef NVBLOX_ROS__TERMINAL_READING_HPP_
#define NVBLOX_ROS__TERMINAL_READING_HPP_

#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

namespace nvblox
{

/// @brief Initialization to read from terminal.
void initTerminal();

/// @brief Reads keys from terminal and integrates the next frame when space
/// key is detected.
/// @return true as long as integrating frames is successful (false if no
/// frames left).
bool readFromTerminal(char * c);

}  // namespace nvblox

#endif  // NVBLOX_ROS__TERMINAL_READING_HPP_
