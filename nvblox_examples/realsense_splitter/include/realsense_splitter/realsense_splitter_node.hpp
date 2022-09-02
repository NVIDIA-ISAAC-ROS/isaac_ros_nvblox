/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef REALSENSE_SPLITTER__REALSENSE_SPLITTER_NODE_HPP_
#define REALSENSE_SPLITTER__REALSENSE_SPLITTER_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <realsense2_camera_msgs/msg/metadata.hpp>

namespace nvblox
{

class RealsenseSplitterNode : public rclcpp::Node
{
public:
  explicit RealsenseSplitterNode(const rclcpp::NodeOptions & options);
  virtual ~RealsenseSplitterNode() = default;

  enum class EmitterMode : int
  {
    kOn = 1,
    kOff = 0
  };

  // Input callbacks
  void image1Callback(
    sensor_msgs::msg::Image::ConstSharedPtr image,
    realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata);
  void image2Callback(
    sensor_msgs::msg::Image::ConstSharedPtr image,
    realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata);
  void depthCallback(
    sensor_msgs::msg::Image::ConstSharedPtr image,
    realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata);
  void pointcloudCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr image,
    realsense2_camera_msgs::msg::Metadata::ConstSharedPtr metadata);

private:
  // Extract the emitter metadata mode
  int getEmitterModeFromMetadataMsg(
    const realsense2_camera_msgs::msg::Metadata::ConstSharedPtr & metadata);

  // Republish the image if the emitter is off
  template<typename MessageType>
  void republishIfEmitterMode(
    const typename MessageType::ConstSharedPtr & image,
    const realsense2_camera_msgs::msg::Metadata::ConstSharedPtr & metadata,
    const EmitterMode emitter_mode,
    typename rclcpp::Publisher<MessageType>::SharedPtr & publisher);

  // Time Sync
  typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::Image, realsense2_camera_msgs::msg::Metadata>
    image_time_policy_t;
  typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::PointCloud2, realsense2_camera_msgs::msg::Metadata>
    pointcloud_time_policy_t;

  // Image subscribers
  std::shared_ptr<message_filters::Synchronizer<image_time_policy_t>>
  timesync_infra_1_;
  message_filters::Subscriber<sensor_msgs::msg::Image> infra_1_sub_;
  message_filters::Subscriber<realsense2_camera_msgs::msg::Metadata>
  infra_1_metadata_sub_;
  std::shared_ptr<message_filters::Synchronizer<image_time_policy_t>>
  timesync_infra_2_;
  message_filters::Subscriber<sensor_msgs::msg::Image> infra_2_sub_;
  message_filters::Subscriber<realsense2_camera_msgs::msg::Metadata>
  infra_2_metadata_sub_;
  std::shared_ptr<message_filters::Synchronizer<image_time_policy_t>>
  timesync_depth_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<realsense2_camera_msgs::msg::Metadata>
  depth_metadata_sub_;
  std::shared_ptr<message_filters::Synchronizer<pointcloud_time_policy_t>>
  timesync_pointcloud_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_sub_;
  message_filters::Subscriber<realsense2_camera_msgs::msg::Metadata>
  pointcloud_metadata_sub_;

  // Image publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr infra_1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr infra_2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

}  // namespace nvblox

#endif  // REALSENSE_SPLITTER__REALSENSE_SPLITTER_NODE_HPP_
