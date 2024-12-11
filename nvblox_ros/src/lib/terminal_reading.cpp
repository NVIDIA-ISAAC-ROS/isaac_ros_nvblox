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

#include <rclcpp/rclcpp.hpp>

#include "nvblox_ros/terminal_reading.hpp"

namespace nvblox
{

// Access to terminal.
int tty_fd;
struct termios current_attributes, modified_attributes;

void handleError(const char * msg)
{
  // Print last error message together with custom message.
  perror(msg);
  exit(1);
}

void revertTerminalSettings(int sig)
{
  (void)sig;
  // Restore the terminal attributes.
  if (tcsetattr(tty_fd, TCSANOW, &current_attributes) < 0) {
    handleError("Failed restoring terminal attributes");
  }
  rclcpp::shutdown();
  exit(0);
}

void initTerminal()
{
  // Open /dev/tty to ensure we are interacting with the terminal.
  tty_fd = open("/dev/tty", O_RDWR);
  if (tty_fd < 0) {
    handleError("Failed to open /dev/tty");
  }

  // Change descriptor to non-blocking.
  int flags = fcntl(tty_fd, F_GETFL, 0);
  if (fcntl(tty_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    handleError("Failed to set descriptor to non-blocking");
  }

  // Set up the signal handler for SIGINT to revert the changes to the terminal
  // attributes when we finish.
  signal(SIGINT, revertTerminalSettings);

  // Obtain the current terminal attributes and store them in
  // 'current_attributes' This will allow us to restore the terminal to its
  // original state later.
  if (tcgetattr(tty_fd, &current_attributes) < 0) {
    handleError("Failed to obtain terminal attributes");
  }

  // Copy the current terminal attributes into so we can modify them.
  memcpy(&modified_attributes, &current_attributes, sizeof(struct termios));

  // Disabling canonical mode to make input available immediately
  // (character-by-character) rather than waiting for a newline.
  modified_attributes.c_lflag &= ~(ICANON);
  // Disabling printing characters as they are typed.
  modified_attributes.c_lflag &= ~(ICANON);

  // Apply the modified terminal attributes.
  if (tcsetattr(tty_fd, TCSANOW, &modified_attributes) < 0) {
    handleError("Failed to apply terminal attributes");
  }
}

bool readFromTerminal(char * c)
{
  // Get the next event from the keyboard.
  if (::read(tty_fd, c, 1) < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // No key entered. Try again later.
    } else {
      handleError("Failed to read from terminal");
    }
    return false;
  }
  return true;
}

}  // namespace nvblox
