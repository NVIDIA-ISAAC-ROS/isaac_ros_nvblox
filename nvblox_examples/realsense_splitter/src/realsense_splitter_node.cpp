// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "realsense_splitter/realsense_splitter_node.hpp"

#include <nvblox_ros_common/qos.hpp>

namespace nvblox
{

RealsenseSplitterNode::RealsenseSplitterNode(const rclcpp::NodeOptions & options)
: Node("realsense_splitter_node", options)
{
  RCLCPP_INFO(get_logger(), "Creating a RealsenseSplitterNode().");

  // Parameters
  const std::string kDefaultQoS = "SYSTEM_DEFAULT";
  std::string input_qos_str =
    declare_parameter<std::string>("input_qos", kDefaultQoS);
  std::string output_qos_str =
    declare_parameter<std::string>("output_qos", kDefaultQoS);

  // Subscribe to synchronized depth + cam_info topics
  const auto input_qos = parseQosString(input_qos_str);
  infra_1_sub_.subscribe(this, "input/infra_1", input_qos);
  infra_1_metadata_sub_.subscribe(this, "input/infra_1_metadata", input_qos);
  infra_2_sub_.subscribe(this, "input/infra_2", input_qos);
  infra_2_metadata_sub_.subscribe(this, "input/infra_2_metadata", input_qos);
  depth_sub_.subscribe(this, "input/depth", input_qos);
  depth_metadata_sub_.subscribe(this, "input/depth_metadata", input_qos);
  pointcloud_sub_.subscribe(this, "input/pointcloud", input_qos);
  pointcloud_metadata_sub_.subscribe(this, "input/pointcloud_metadata", input_qos);

  constexpr int kInputQueueSize = 10;
  timesync_infra_1_.reset(
    new message_filters::Synchronizer<image_time_policy_t>(
      image_time_policy_t(kInputQueueSize), infra_1_sub_,
      infra_1_metadata_sub_));
  timesync_infra_1_->registerCallback(
    std::bind(
      &RealsenseSplitterNode::image1Callback, this,
      std::placeholders::_1, std::placeholders::_2));
  timesync_infra_2_.reset(
    new message_filters::Synchronizer<image_time_policy_t>(
      image_time_policy_t(kInputQueueSize), infra_2_sub_,
      infra_2_metadata_sub_));
  timesync_infra_2_->registerCallback(
    std::bind(
      &RealsenseSplitterNode::image2Callback, this,
      std::placeholders::_1, std::placeholders::_2));
  timesync_depth_.reset(
    new message_filters::Synchronizer<image_time_policy_t>(
      image_time_policy_t(kInputQueueSize), depth_sub_, depth_metadata_sub_));
  timesync_depth_->registerCallback(
    std::bind(
      &RealsenseSplitterNode::depthCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  timesync_pointcloud_.reset(
    new message_filters::Synchronizer<pointcloud_time_policy_t>(
      pointcloud_time_policy_t(kInputQueueSize), pointcloud_sub_,
      pointcloud_metadata_sub_));
  timesync_pointcloud_->registerCallback(
    std::bind(
      &RealsenseSplitterNode::pointcloudCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  // Publisher
  constexpr size_t kOutputQueueSize = 10;
  const auto output_qos = rclcpp::QoS(
    rclcpp::KeepLast(kOutputQueueSize),
    parseQosString(output_qos_str));
  infra_1_pub_ =
    create_publisher<sensor_msgs::msg::Image>("~/output/infra_1", output_qos);
  infra_2_pub_ =
    create_publisher<sensor_msgs::msg::Image>("~/output/infra_2", output_qos);
  depth_pub_ =
    create_publisher<sensor_msgs::msg::Image>("~/output/depth", output_qos);
  pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/output/pointcloud", output_qos);
}

int RealsenseSplitterNode::getEmitterModeFromMetadataMsg(
  const realsense2_camera_msgs::msg::Metadata::ConstSharedPtr & metadata)
{
  // Field name in json metadata
  constexpr char frame_emitter_mode_str[] = "\"frame_emitter_mode\":";
  constexpr size_t field_name_length =
    sizeof(frame_emitter_mode_str) / sizeof(frame_emitter_mode_str[0]);
  // Find the field
  const size_t frame_emitter_mode_start_location =
    metadata->json_data.find(frame_emitter_mode_str);
  // If the emitter mode is not found, return unknown and warn the user.
  if (frame_emitter_mode_start_location == metadata->json_data.npos) {
    constexpr int kPublishPeriodMs = 1000;
    auto & clk = *get_clock();
    RCLCPP_WARN_THROTTLE(
      get_logger(), clk, kPublishPeriodMs,
      "Realsense frame metadata did not contain \"frame_emitter_mode\". Splitter will not work.");
    return static_cast<int>(EmitterMode::kUnknown);
  }
  // If it is found, parse the field.
  const size_t field_location = frame_emitter_mode_start_location + field_name_length - 1;
  const int emitter_mode =
    static_cast<int>(metadata->json_data[field_location]) -
    static_cast<int>('0');
  return emitter_mode;
}

template<typename MessageType>
void RealsenseSplitterNode::republishIfEmitterMode(
  const typename MessageType::ConstSharedPtr & image,
  const realsense2_camera_msgs::msg::Metadata::ConstSharedPtr & metadata,
  const EmitterMode emitter_mode,
  typename rclcpp::Publisher<MessageType>::SharedPtr & publisher)
{
  if (getEmitterModeFromMetadataMsg(metadata) ==
    static_cast<int>(emitter_mode))
  {
    publisher->publish(*image);
  }
}

void RealsenseSplitterNode::image1Callback(
  sensor_msgs::msg::Image::ConstSharedPtr image,
  realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata)
{
  republishIfEmitterMode<sensor_msgs::msg::Image>(
    image, metadata, EmitterMode::kOff, infra_1_pub_);
}

void RealsenseSplitterNode::image2Callback(
  sensor_msgs::msg::Image::ConstSharedPtr image,
  realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata)
{
  republishIfEmitterMode<sensor_msgs::msg::Image>(
    image, metadata, EmitterMode::kOff, infra_2_pub_);
}

void RealsenseSplitterNode::depthCallback(
  sensor_msgs::msg::Image::ConstSharedPtr image,
  realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata)
{
  republishIfEmitterMode<sensor_msgs::msg::Image>(
    image, metadata,
    EmitterMode::kOn, depth_pub_);
}

void RealsenseSplitterNode::pointcloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud,
  realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata)
{
  republishIfEmitterMode<sensor_msgs::msg::PointCloud2>(
    pointcloud, metadata, EmitterMode::kOn, pointcloud_pub_);
}

}  // namespace nvblox

// Register the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvblox::RealsenseSplitterNode)
