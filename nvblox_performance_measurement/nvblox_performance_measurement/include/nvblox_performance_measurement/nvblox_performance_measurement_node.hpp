/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_NODE_HPP_
#define NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_NODE_HPP_

#include <std_msgs/msg/string.hpp>

#include <nvblox_ros/nvblox_node.hpp>

#include <nvblox_performance_measurement_msgs/msg/frame_processed.hpp>

namespace nvblox
{

class NvbloxPerformanceMeasurementNode : public NvbloxNode
{
public:
  NvbloxPerformanceMeasurementNode();
  virtual ~NvbloxPerformanceMeasurementNode() = default;

  // Callback functions. These just stick images in a queue.
  // Process a single images
  virtual bool processDepthImage(
    sensor_msgs::msg::Image::ConstSharedPtr & depth_img_ptr,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg) override;
  virtual bool processColorImage(
    sensor_msgs::msg::Image::ConstSharedPtr & color_img_ptr,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg) override;
  virtual bool processLidarPointcloud(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_ptr) override;

  virtual void processMesh() override;

  void publishTimersCallback();

private:
  // Parameters
  float timer_publish_rate_hz_ = 1.0;

  // Publishers
  rclcpp::Publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed>::SharedPtr
    depth_processed_publisher_;
  rclcpp::Publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed>::SharedPtr
    color_processed_publisher_;
  rclcpp::Publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed>::SharedPtr
    pointcloud_processed_publisher_;
  rclcpp::Publisher<nvblox_performance_measurement_msgs::msg::FrameProcessed>::SharedPtr
    mesh_processed_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timers_publisher_;

  // Timers
  rclcpp::TimerBase::SharedPtr timers_publishing_timer_;
};

}  // namespace nvblox

#endif  // NVBLOX_PERFORMANCE_MEASUREMENT__NVBLOX_PERFORMANCE_MEASUREMENT_NODE_HPP_
