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

#include <string>

#include <isaac_ros_common/qos.hpp>

#include "nvblox_image_padding/image_padding_cropping_node.hpp"

namespace nvblox
{

using std::placeholders::_1;

ImagePaddingCroppingNode::ImagePaddingCroppingNode(
  const rclcpp::NodeOptions & options,
  const std::string & node_name)
: Node(node_name, options)
{
  RCLCPP_INFO(get_logger(), "Starting up ImagePaddingCroppingNode");

  // Settings for QoS.
  const rclcpp::QoS image_qos =
    isaac_ros::common::AddQosParameter(*this, kDefaultImageQos_, "image_qos");

  desired_height_ = declare_parameter<int>("desired_height", desired_height_);
  desired_width_ = declare_parameter<int>("desired_width", desired_width_);

  // Users like feedback!
  if (desired_height_ <= 0 || desired_width_ <= 0) {
    RCLCPP_FATAL_STREAM(
      get_logger(),
      "Parameters \"desired_height\" and \"desired_width\" need both to be "
      "set and > 0. Currently desired_height="
        << desired_height_ << " and desired_width=" << desired_width_);
    exit(1);
  }
  RCLCPP_INFO_STREAM(
    get_logger(), "Cropping/padding to desired_height="
      << desired_height_
      << ", and to desired_width=" << desired_width_);

  // Subscriptions
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "~/image_in", image_qos, std::bind(&ImagePaddingCroppingNode::imageCallback, this, _1));

  // Publishers
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_out", image_qos);
}

void ImagePaddingCroppingNode::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr image_ptr)
{
  // Determine if we're cropping or padding.
  // NOTE(alexmillane): This is not totally general, if the user requests a pad
  // in one dimension and a crop in the other we just fail. Improving this is an
  // exercise for the reader.
  // Determine if cropping is needed in width and/or height
  // Included logic incase one of the dimensions are equal to their corresponding desired value
  const bool crop_in_width = desired_width_ < static_cast<int>(image_ptr->width);
  const bool crop_in_height = desired_height_ < static_cast<int>(image_ptr->height);

  // Ensure consistency: either crop both dimensions or neither, when not matching desired sizes
  assert(
    !(desired_width_ != static_cast<int>(image_ptr->width) &&
    desired_height_ != static_cast<int>(image_ptr->height)) ||
    (crop_in_width == crop_in_height));

  // Determine the action (crop or pad) based on the dimension that does not match the desired size
  // If both dimensions match their desired sizes, default to cropping or padding based on width
  // (arbitrary choice)
  bool crop_if_true_pad_if_false =
    (desired_width_ == static_cast<int>(image_ptr->width)) ? crop_in_height : crop_in_width;

  // Calculate the crop/pad amount
  const int left_pixels =
    std::abs(static_cast<int>((desired_width_ - static_cast<int>(image_ptr->width)) / 2.0));
  const int top_pixels =
    std::abs(static_cast<int>((desired_height_ - static_cast<int>(image_ptr->height)) / 2.0));

  if (crop_if_true_pad_if_false) {
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "Cropping left by: " << left_pixels);
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "Cropping top by: " << top_pixels);
  } else {
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "Padding left by: " << left_pixels);
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "Padding top by: " << top_pixels);
  }

  // Warn if the crop/pad amount is fractional
  if (((desired_width_ - image_ptr->width) % 2) != 0) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "Desired width - Image width not cleanly dividable by two. Extra pixel "
      "will be made up on one side.");
  }
  if (((desired_height_ - image_ptr->height) % 2) != 0) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "Desired height - Image height not cleanly dividable by two. Extra "
      "pixel will be made up on one side.");
  }

  // Wrap input image
  cv_bridge::CvImageConstPtr input_image_cv_ptr =
    cv_bridge::toCvShare(image_ptr, image_ptr->encoding);

  // Prepare empty output image
  cv_bridge::CvImage output_image_cv(input_image_cv_ptr->header, input_image_cv_ptr->encoding);

  // Crop (if that's what we want)
  if (crop_if_true_pad_if_false) {
    output_image_cv.image = input_image_cv_ptr->image(
      cv::Rect(left_pixels, top_pixels, desired_width_, desired_height_));
  }
  // Pad (if that's what we want)
  else {
    output_image_cv.image =
      cv::Mat::zeros(desired_height_, desired_width_, input_image_cv_ptr->image.type());
    input_image_cv_ptr->image.copyTo(
      output_image_cv.image(
        cv::Rect(left_pixels, top_pixels, image_ptr->width, image_ptr->height)));
  }
  // Republish
  image_pub_->publish(*output_image_cv.toImageMsg());
}

}  // namespace nvblox

// Register the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvblox::ImagePaddingCroppingNode)
