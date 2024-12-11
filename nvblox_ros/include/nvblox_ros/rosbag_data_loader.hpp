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

#ifndef NVBLOX_ROS__ROSBAG_DATA_LOADER_HPP_
#define NVBLOX_ROS__ROSBAG_DATA_LOADER_HPP_

#include <memory>
#include <optional>
#include <string>

#include "tf2_ros/buffer.h"

#include "nvblox/datasets/data_loader.h"
#include "nvblox/executables/fuser.h"
#include "nvblox/nvblox.h"

#include "rclcpp/logger.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "nvblox_ros/conversions/image_conversions_thrust.hpp"

namespace nvblox
{
namespace datasets
{
namespace ros
{

/// Builds a Fuser that returns nvblox data from a ROSbag.
/// @param rosbag_path The path to the ROSbag.
/// @param depth_topic Name of the depth topic.
/// @param depth_camera_info_topic Name of the camera_info topic associated
/// with the depth topic.
/// @param color_topic Name of the color image topic.
/// @param color_camera_info_topic Name of the camera_info topic associated
/// with the color topic.
/// @param global_frame_id The camera of the frame that returned camera poses
/// are expressed with respect to.
/// @param tf_preload_time_s The amount of seconds that we load /tf messages
/// in the tf_buffer in advance of loading the image topics.
/// @param cuda_stream The CUDA stream on which to perform operations.
/// @return std::unique_ptr<Fuser> A fuser fusing ROSbag data. May be nullptr if
/// construction fails.
std::unique_ptr<Fuser> createFuser(
  const std::string & rosbag_path,               // NOLINT
  const std::string & depth_topic,               // NOLINT
  const std::string & depth_camera_info_topic,   // NOLINT
  const std::string & color_topic,               // NOLINT
  const std::string & color_camera_info_topic,   // NOLINT
  const std::string & global_frame_id,           // NOLINT
  const float tf_preload_time_s,                 // NOLINT
  std::shared_ptr<CudaStream> cuda_stream =
  std::make_shared<CudaStreamOwning>());

/// An nvblox data loader which loads data from a ROSbag.
/// Note that at the moment the loader expects *exact* timestamp matches between
/// depth, color, depth camera_info, and color camera_info topics.
/// /tf_static is loaded at the startup. /tf is loaded into the TFBuffer such
/// that it preceeds the images by a configurable amount of time.
class RosDataLoader : public datasets::RgbdDataLoaderInterface
{
public:
  /// Construct a data loader that returns nvblox data from a ROSbag.
  /// @param rosbag_path The path to the ROSbag.
  /// @param depth_topic Name of the depth topic.
  /// @param depth_camera_info_topic Name of the camera_info topic associated
  /// with the depth topic.
  /// @param color_topic Name of the color image topic.
  /// @param color_camera_info_topic Name of the camera_info topic associated
  /// with the color topic.
  /// @param global_frame_id The camera of the frame that returned camera poses
  /// are expressed with respect to.
  /// @param tf_preload_time_s The amount of seconds that we load /tf messages
  /// in the tf_buffer in advance of loading the image topics.
  /// @param cuda_stream The CUDA stream on which to perform operations.
  RosDataLoader(
    const std::string & rosbag_path,                         // NOLINT
    const std::string & depth_topic,                         // NOLINT
    const std::string & depth_camera_info_topic,             // NOLINT
    const std::string & color_topic,                         // NOLINT
    const std::string & color_camera_info_topic,             // NOLINT
    const std::string & global_frame_id,                     // NOLINT
    const float tf_preload_time_s,                           // NOLINT
    std::shared_ptr<CudaStream> cuda_stream =
    std::make_shared<CudaStreamOwning>());
  virtual ~RosDataLoader() = default;

  /// Builds a DatasetLoader that returns nvblox data from a ROSbag.
  /// @param rosbag_path The path to the ROSbag.
  /// @param depth_topic Name of the depth topic.
  /// @param depth_camera_info_topic Name of the camera_info topic associated
  /// with the depth topic.
  /// @param color_topic Name of the color image topic.
  /// @param color_camera_info_topic Name of the camera_info topic associated
  /// with the color topic.
  /// @param global_frame_id The camera of the frame that returned camera poses
  /// are expressed with respect to.
  /// @param tf_preload_time_s The amount of seconds that we load /tf messages
  /// in the tf_buffer in advance of loading the image topics.
  /// @param cuda_stream The CUDA stream on which to perform operations.
  /// @return std::unique_ptr<RosDataLoader> The dataset loader. May be nullptr
  /// if construction fails.
  static std::unique_ptr<RosDataLoader> create(
    const std::string & rosbag_path,               // NOLINT
    const std::string & depth_topic,               // NOLINT
    const std::string & depth_camera_info_topic,   // NOLINT
    const std::string & color_topic,               // NOLINT
    const std::string & color_camera_info_topic,   // NOLINT
    const std::string & global_frame_id,           // NOLINT
    const float tf_preload_time_s,                 // NOLINT
    std::shared_ptr<CudaStream> cuda_stream =
    std::make_shared<CudaStreamOwning>());

  /// Interface for a function that loads the next frames in a dataset
  /// This version of the function should be used when the color and depth
  /// camera are the same.
  /// @param[out] depth_frame_ptr The loaded depth frame.
  /// @param[out] T_L_C_ptr Transform from Camera to the Layer frame.
  /// @param[out] camera_ptr The intrinsic camera model.
  /// @param[out] color_frame_ptr Optional, load color frame.
  /// @return Whether loading succeeded.
  DataLoadResult loadNext(
    DepthImage * depth_frame_ptr,                       // NOLINT
    Transform * T_L_C_ptr,                              // NOLINT
    Camera * camera_ptr,                                // NOLINT
    ColorImage * color_frame_ptr = nullptr) override;

  /// Interface for a function that loads the next frames in a dataset.
  /// This is the version of the function for different depth and color cameras.
  /// @param[out] depth_frame_ptr The loaded depth frame.
  /// @param[out] T_L_D_ptr Transform from depth camera to the Layer frame.
  /// @param[out] depth_camera_ptr The intrinsic depth camera model.
  /// @param[out] color_frame_ptr The loaded color frame.
  /// @param[out] T_L_C_ptr Transform from color camera to the Layer frame.
  /// @param[out] color_camera_ptr The intrinsic color camera model.
  /// @return Whether loading succeeded.
  DataLoadResult loadNext(
    DepthImage * depth_frame_ptr,                       // NOLINT
    Transform * T_L_D_ptr,                              // NOLINT
    Camera * depth_camera_ptr,                          // NOLINT
    ColorImage * color_frame_ptr,                       // NOLINT
    Transform * T_L_C_ptr,                              // NOLINT
    Camera * color_camera_ptr) override;

private:
  // Steps all message streams forward until we have a match.
  bool stepUntilNextMatchingMessages();
  // Step individual message streams forward by a single message.
  bool stepDepthMsg(sensor_msgs::msg::Image * msg_ptr);
  bool stepDepthCameraInfoMsg(sensor_msgs::msg::CameraInfo * msg_ptr);
  bool stepColorMsg(sensor_msgs::msg::Image * msg_ptr);
  bool stepColorCameraInfoMsg(sensor_msgs::msg::CameraInfo * msg_ptr);
  bool stepTfMsg(tf2_msgs::msg::TFMessage * msg_ptr);

  // Does the combination of messages represent a timestamp match which can be
  // returned from the data loader to the caller.
  bool isMatch(
    const sensor_msgs::msg::Image & depth_msg,                    // NOLINT
    const sensor_msgs::msg::CameraInfo & depth_camera_info_msg,   // NOLINT
    const sensor_msgs::msg::Image & color_msg,                    // NOLINT
    const sensor_msgs::msg::CameraInfo & color_camera_info_msg,   // NOLINT
    const tf2_msgs::msg::TFMessage & tf_msg);

  // "World"/Layer frame_id. This is the frame which cameras poses are expressed
  // with respect to.
  std::string global_frame_id_;

  // Tf buffer
  tf2_ros::Buffer tf_buffer_;

  // ROS logging.
  rclcpp::Logger ros_logger_;

  // Single topic rosbag readers
  rosbag2_cpp::Reader depth_reader_;
  rosbag2_cpp::Reader color_reader_;
  rosbag2_cpp::Reader depth_camera_info_reader_;
  rosbag2_cpp::Reader color_camera_info_reader_;
  rosbag2_cpp::Reader tf_reader_;

  // The last read messages
  std::optional<sensor_msgs::msg::Image> depth_image_msg_;
  std::optional<sensor_msgs::msg::CameraInfo> depth_camera_info_msg_;
  std::optional<sensor_msgs::msg::Image> color_image_msg_;
  std::optional<sensor_msgs::msg::CameraInfo> color_camera_info_msg_;
  std::optional<tf2_msgs::msg::TFMessage> tf_msg_;

  // Scratch for depth image conversion.
  Image<int16_t> depth_image_conversion_scratch_;
  Image<conversions::Bgra> bgra_image_conversion_scratch_;
  Image<conversions::Rgb> rgb_image_conversion_scratch_;

  // We preload the /tf topic into the TFBuffer by this amount of seconds in
  // advance of loading the the image streams. This is to try to guarantee that
  // the transform is available.
  float tf_lead_time_s_ = 0.5;

  // CUDA stream on which to process operations.
  std::shared_ptr<CudaStream> cuda_stream_;
};

}  // namespace ros
}  // namespace datasets
}  // namespace nvblox

#endif  // NVBLOX_ROS__ROSBAG_DATA_LOADER_HPP_
