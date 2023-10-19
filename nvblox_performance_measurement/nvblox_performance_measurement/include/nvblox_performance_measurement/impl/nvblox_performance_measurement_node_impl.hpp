// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_NODE_IMPL_HPP_
#define NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_NODE_IMPL_HPP_

#include "nvblox_performance_measurement/nvblox_performance_measurement_node.hpp"

#include "nvblox_performance_measurement/nvblox_performance_common.hpp"

namespace nvblox
{

template<typename NvbloxType, typename MessageTupleType>
NvbloxPerformanceMeasurementNodeTemplate<NvbloxType,
  MessageTupleType>::NvbloxPerformanceMeasurementNodeTemplate(
  const rclcpp::NodeOptions & options)
: NvbloxType(options)
{
  // Parameters
  timer_publish_rate_hz_ =
    NvbloxType::template declare_parameter<float>("timer_publish_rate_hz", timer_publish_rate_hz_);

  // Publishers
  depth_processed_publisher_ = NvbloxType::template create_publisher<
    nvblox_performance_measurement_msgs::msg::FrameProcessed>(
    "~/depth_processed", 10);
  color_processed_publisher_ = NvbloxType::template create_publisher<
    nvblox_performance_measurement_msgs::msg::FrameProcessed>(
    "~/color_processed", 10);
  pointcloud_processed_publisher_ = NvbloxType::template create_publisher<
    nvblox_performance_measurement_msgs::msg::FrameProcessed>(
    "~/pointcloud_processed", 10);
  mesh_processed_publisher_ = NvbloxType::template create_publisher<
    nvblox_performance_measurement_msgs::msg::FrameProcessed>(
    "~/mesh_processed", 10);
  timers_publisher_ = NvbloxType::template create_publisher<std_msgs::msg::String>("~/timers", 10);

  // Timers
  timers_publishing_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / timer_publish_rate_hz_),
    std::bind(
      &NvbloxPerformanceMeasurementNodeTemplate<NvbloxType,
      MessageTupleType>::publishTimersCallback,
      this));
}

template<typename NvbloxType, typename MessageTupleType>
bool NvbloxPerformanceMeasurementNodeTemplate<NvbloxType, MessageTupleType>::processDepthImage(
  const MessageTupleType & depth_mask_msg)
{
  // Process as declared in NvbloxHumanNode + emit a header
  auto process_msg_functor =
    [this](const MessageTupleType & msg) -> bool {
      return NvbloxType::processDepthImage(msg);
    };
  auto get_header_functor = [](const MessageTupleType & msg) {
      return std::get<0>(msg)->header;
    };
  return callProcessMessageAndEmit<MessageTupleType>(
    process_msg_functor, depth_mask_msg, get_header_functor,
    depth_processed_publisher_.get());
}

template<typename NvbloxType, typename MessageTupleType>
bool NvbloxPerformanceMeasurementNodeTemplate<NvbloxType, MessageTupleType>::processColorImage(
  const MessageTupleType & color_mask_msg)
{
  // Process as declared in NvbloxHumanNode + emit a header
  auto process_msg_functor =
    [this](const MessageTupleType & msg) -> bool {
      return NvbloxType::processColorImage(msg);
    };
  auto get_header_functor = [](const MessageTupleType & msg) {
      return std::get<0>(msg)->header;
    };
  return callProcessMessageAndEmit<MessageTupleType>(
    process_msg_functor, color_mask_msg, get_header_functor,
    color_processed_publisher_.get());
}

template<typename NvbloxType, typename MessageTupleType>
bool NvbloxPerformanceMeasurementNodeTemplate<NvbloxType, MessageTupleType>::processLidarPointcloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_ptr)
{
  // Process as declared in NvbloxHumanNode + emit a header
  auto process_msg_functor =
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) -> bool {
      return NvbloxType::processLidarPointcloud(msg);
    };
  auto get_header_functor =
    [](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
      return msg->header;
    };
  return callProcessMessageAndEmit<
    sensor_msgs::msg::PointCloud2::ConstSharedPtr>(
    process_msg_functor, pointcloud_ptr, get_header_functor,
    pointcloud_processed_publisher_.get());
}

template<typename NvbloxType, typename MessageTupleType>
void NvbloxPerformanceMeasurementNodeTemplate<NvbloxType, MessageTupleType>::processMesh()
{
  // Process as declared in NvbloxHumanNode + emit a header
  auto process = [this]() {NvbloxType::processMesh();};
  callAndEmit(process, NvbloxType::get_clock()->now(), mesh_processed_publisher_.get());
}

template<typename NvbloxType, typename MessageTupleType>
void NvbloxPerformanceMeasurementNodeTemplate<NvbloxType, MessageTupleType>::publishTimersCallback()
{
  std_msgs::msg::String msg;
  msg.data = nvblox::timing::Timing::Print();
  timers_publisher_->publish(msg);
}

}  // namespace nvblox

#endif  // NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_NODE_IMPL_HPP_
