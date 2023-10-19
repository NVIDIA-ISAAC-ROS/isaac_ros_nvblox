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

#ifndef NVBLOX_ROS__NVBLOX_HUMAN_NODE_HPP_
#define NVBLOX_ROS__NVBLOX_HUMAN_NODE_HPP_

#include <nvblox/mapper/multi_mapper.h>
#include <nvblox/semantics/image_projector.h>
#include <nvblox/sensors/pointcloud.h>

#include <deque>
#include <memory>
#include <tuple>

#include "nvblox_ros/nvblox_node.hpp"

namespace nvblox
{

class NvbloxHumanNode : public NvbloxNode
{
public:
  explicit NvbloxHumanNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~NvbloxHumanNode() = default;

  // Setup. These are called by the constructor.
  void getParameters();
  void subscribeToTopics();
  void setupTimers();
  void advertiseTopics();

  // Callbacks for Sensor + Mask
  void depthPlusMaskImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_img_ptr,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & mask_img_ptr,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & mask_camera_info_msg);
  void colorPlusMaskImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color_img_ptr,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & mask_img_ptr,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & mask_camera_info_msg);

  // This is our internal type for passing around images, their matching
  // segmentation masks, as well as the camera intrinsics.
  using ImageSegmentationMaskMsgTuple =
    std::tuple<sensor_msgs::msg::Image::ConstSharedPtr,
      sensor_msgs::msg::CameraInfo::ConstSharedPtr,
      sensor_msgs::msg::Image::ConstSharedPtr,
      sensor_msgs::msg::CameraInfo::ConstSharedPtr>;

  // Override the depth processing from the base node
  void processDepthQueue() override;
  void processColorQueue() override;

  // The methods for processing images from the internal queue.
  virtual bool processDepthImage(
    const ImageSegmentationMaskMsgTuple & depth_mask_msg);
  virtual bool processColorImage(
    const ImageSegmentationMaskMsgTuple & color_mask_msg);

protected:
  // Publish human data (if any subscribers) that helps
  // visualization and debugging.
  void publishHumanDebugOutput();

  // Approx Synchronize: Depth + CamInfo + SegmentationMake + CamInfo
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
      sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>
    mask_approximate_time_policy_t;

  // Exact Synchronize: Color + CamInfo + SegmentationMake + CamInfo
  typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
      sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>
    mask_exact_time_policy_t;

  std::shared_ptr<message_filters::Synchronizer<mask_approximate_time_policy_t>>
  timesync_depth_mask_;
  std::shared_ptr<message_filters::Synchronizer<mask_exact_time_policy_t>>
  timesync_color_mask_;

  // Segmentation mask sub.
  message_filters::Subscriber<sensor_msgs::msg::Image> segmentation_mask_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo>
  segmentation_camera_info_sub_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    human_voxels_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    depth_frame_overlay_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    color_frame_overlay_publisher_;

  // Timers
  rclcpp::TimerBase::SharedPtr human_debug_publish_timer__;

  // Rates.
  float human_debug_publish_rate_hz_ = 10.0f;

  // Image queues.
  // Note these differ from the base class image queues because they also
  // include segmentation images. The base class queue are disused in the
  // NvbloxHumanNode.
  std::deque<ImageSegmentationMaskMsgTuple> depth_mask_image_queue_;
  std::deque<ImageSegmentationMaskMsgTuple> color_mask_image_queue_;

  // Cache for GPU image
  MonoImage mask_image_{MemoryType::kDevice};

  // Image queue mutexes.
  std::mutex depth_mask_queue_mutex_;
  std::mutex color_mask_queue_mutex_;

  // Device caches
  Pointcloud human_pointcloud_C_device_;
  Pointcloud human_pointcloud_L_device_;
  Pointcloud human_voxel_centers_L_device_;

  // Caching data of last depth frame for debug outputs
  Camera depth_camera_;
  Transform T_L_C_depth_;
};

}  // namespace nvblox

#endif  // NVBLOX_ROS__NVBLOX_HUMAN_NODE_HPP_
