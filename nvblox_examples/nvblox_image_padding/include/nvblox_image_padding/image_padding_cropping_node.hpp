// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVBLOX_IMAGE_PADDING__IMAGE_PADDING_NODE_HPP_
#define NVBLOX_IMAGE_PADDING__IMAGE_PADDING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.hpp>

namespace nvblox
{

class ImagePaddingCroppingNode : public rclcpp::Node
{
public:
  explicit ImagePaddingCroppingNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "image_padding_node");
  virtual ~ImagePaddingCroppingNode() = default;

  // Callbacks
  void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr image_ptr);

private:
  // Image subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // Image publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  // The amount to pad (or crop)
  int desired_height_ = -1;
  int desired_width_ = -1;

  // Default subscription QoS
  const std::string kDefaultImageQos_ = "SYSTEM_DEFAULT";
};

} // namespace nvblox

#endif  // NVBLOX_IMAGE_PADDING__IMAGE_PADDING_NODE_HPP_
