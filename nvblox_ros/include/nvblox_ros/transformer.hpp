/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef NVBLOX_ROS__TRANSFORMER_HPP_
#define NVBLOX_ROS__TRANSFORMER_HPP_

#include <nvblox/core/types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace nvblox
{

/// Class that binds to either the TF tree or resolves transformations from the
/// ROS parameter server, depending on settings loaded from ROS params.
class Transformer
{
public:
  explicit Transformer(rclcpp::Node * node);

  bool lookupTransformToGlobalFrame(
    const std::string & sensor_frame,
    const rclcpp::Time & timestamp,
    Transform * transform);

  /// Assumes these transforms are from GLOBAL frame to POSE frame. Ignores
  /// frame_id.
  void transformCallback(
    const geometry_msgs::msg::TransformStamped::ConstSharedPtr transform_msg);
  void poseCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr transform_msg);

  /// Set the names of the frames.
  void set_global_frame(const std::string & global_frame)
  {
    global_frame_ = global_frame;
  }

  void set_pose_frame(const std::string & pose_frame)
  {
    pose_frame_ = pose_frame;
  }

private:
  bool lookupTransformTf(
    const std::string & from_frame,
    const std::string & to_frame,
    const rclcpp::Time & timestamp, Transform * transform);

  bool lookupTransformQueue(
    const rclcpp::Time & timestamp,
    Transform * transform);

  bool lookupSensorTransform(
    const std::string & sensor_frame,
    Transform * transform);

  /// ROS State
  rclcpp::Node * node_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  /// Global/map coordinate frame. Will always look up TF transforms to this
  /// frame.
  std::string global_frame_;
  /// Use this as the "pose" frame that's coming in. Needs to be set.
  std::string pose_frame_;

  /// Whether to use TF at all. If set to false, sensor_frame and pose_frame
  /// *need* to be set.
  bool use_tf_transforms_ = true;
  /// Whether to listen to topics for transforms. If set to true, will get
  /// at least global -> pose frame from the topics. If set to false, everything
  /// will be resolved through TF.
  bool use_topic_transforms_ = false;
  /// Timestamp tolerance to use for transform *topics* only.
  uint64_t timestamp_tolerance_ns_ = 1e8;  // 100 milliseconds

  /// Queues and state
  /// Maps timestamp in ns to transform global -> pose frame.
  std::map<uint64_t, Transform> transform_queue_;
  /// Maps sensor frame to transform pose frame -> sensor frame.
  std::unordered_map<std::string, Transform> sensor_transforms_;
};

}  // namespace nvblox

#endif  // NVBLOX_ROS__TRANSFORMER_HPP_
