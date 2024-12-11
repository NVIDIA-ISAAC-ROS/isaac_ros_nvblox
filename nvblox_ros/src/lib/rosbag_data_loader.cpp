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
#include "nvblox_ros/rosbag_data_loader.hpp"

#include <filesystem>
#include <optional>

#include "glog/logging.h"

#include "nvblox/datasets/data_loader.h"
#include "nvblox/nvblox.h"

#include "rclcpp/clock.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "nvblox_ros/conversions/image_conversions.hpp"
#include "nvblox_ros/rosbag_reading.hpp"

namespace nvblox
{
namespace datasets
{
namespace ros
{

// File-local fuctions
namespace
{

constexpr char kTfStaticTopicName[] = "/tf_static";
constexpr char kTfTopicName[] = "/tf";

template<typename MessageType>
bool stepNextMessage(
  MessageType * msg_ptr, rosbag2_cpp::Reader * reader_ptr,
  const std::string & topic_name = "")
{
  CHECK_NOTNULL(msg_ptr);
  if (!reader_ptr->has_next()) {
    return false;
  }
  if (topic_name.length() > 0) {
    DLOG(INFO) << "Stepping: " << topic_name;
  }
  rosbag2_storage::SerializedBagMessageSharedPtr serialized_msg_ptr = reader_ptr->read_next();
  *msg_ptr = deserializeMessage<MessageType>(*serialized_msg_ptr);
  return true;
}

bool messagesAvailable(rosbag2_cpp::Reader & reader) {return reader.has_next();}

template<typename ... Args> bool messagesAvailable(rosbag2_cpp::Reader & reader, Args &... args)
{
  return reader.has_next() && messagesAvailable(args ...);
}

struct RosTimeLessThan
{
  bool operator()(
    const builtin_interfaces::msg::Time & lhs,
    const builtin_interfaces::msg::Time & rhs)
  {
    return lhs < rhs;
  }
};

}  // namespace

std::unique_ptr<Fuser> createFuser(
  const std::string & rosbag_path,                                              // NOLINT
  const std::string & depth_topic,                                              // NOLINT
  const std::string & depth_camera_info_topic,                                  // NOLINT
  const std::string & color_topic,                                              // NOLINT
  const std::string & color_camera_info_topic,                                  // NOLINT
  const std::string & global_frame_id,                                          // NOLINT
  const float tf_preload_time_s,                                                // NOLINT
  std::shared_ptr<CudaStream> cuda_stream)
{
  auto data_loader = RosDataLoader::create(
    rosbag_path, depth_topic, depth_camera_info_topic,
    color_topic, color_camera_info_topic, global_frame_id,
    tf_preload_time_s, cuda_stream);
  if (!data_loader) {
    return std::unique_ptr<Fuser>();
  }
  constexpr bool kDontInitializeFromGflagsFlag = false;
  return std::make_unique<Fuser>(std::move(data_loader), kDontInitializeFromGflagsFlag);
}

std::unique_ptr<RosDataLoader>
RosDataLoader::create(
  const std::string & rosbag_path,                                 // NOLINT
  const std::string & depth_topic,                                 // NOLINT
  const std::string & depth_camera_info_topic,                     // NOLINT
  const std::string & color_topic,                                 // NOLINT
  const std::string & color_camera_info_topic,                     // NOLINT
  const std::string & global_frame_id,                             // NOLINT
  const float tf_preload_time_s,                                   // NOLINT
  std::shared_ptr<CudaStream> cuda_stream)
{
  // Construct a dataset loader but only return it if everything worked.
  auto dataset_loader = std::make_unique<RosDataLoader>(
    rosbag_path, depth_topic, depth_camera_info_topic, color_topic, color_camera_info_topic,
    global_frame_id, tf_preload_time_s, cuda_stream);
  if (dataset_loader->setup_success_) {
    return dataset_loader;
  } else {
    return std::unique_ptr<RosDataLoader>();
  }
}

RosDataLoader::RosDataLoader(
  const std::string & rosbag_path, const std::string & depth_topic,
  const std::string & depth_camera_info_topic,
  const std::string & color_topic,
  const std::string & color_camera_info_topic,
  const std::string & global_frame_id, const float tf_preload_time_s,
  std::shared_ptr<CudaStream> cuda_stream)
: RgbdDataLoaderInterface(), global_frame_id_(global_frame_id),
  tf_buffer_(std::make_shared<rclcpp::Clock>()),
  ros_logger_(rclcpp::get_logger("RosDataLoader")),
  depth_image_conversion_scratch_(MemoryType::kDevice),
  bgra_image_conversion_scratch_(MemoryType::kDevice),
  rgb_image_conversion_scratch_(MemoryType::kDevice), tf_lead_time_s_(tf_preload_time_s),
  cuda_stream_(cuda_stream)
{
  if (!std::filesystem::exists(rosbag_path)) {
    LOG(WARNING) << "The input rosbag path does not exist: " << rosbag_path;
    setup_success_ = false;
  }

  // Initialize rosbag readers for each topic
  createSingleTopicReader(rosbag_path, depth_topic, &depth_reader_);
  createSingleTopicReader(rosbag_path, depth_camera_info_topic, &depth_camera_info_reader_);
  createSingleTopicReader(rosbag_path, color_topic, &color_reader_);
  createSingleTopicReader(rosbag_path, color_camera_info_topic, &color_camera_info_reader_);
  createSingleTopicReader(rosbag_path, kTfTopicName, &tf_reader_);
  // Load all the static transforms
  loadAllTfStaticsIntoBuffer(rosbag_path, &tf_buffer_);

  // Check topics have messages or indicate failure
  if (!depth_reader_.has_next() || !depth_camera_info_reader_.has_next() ||
    !color_reader_.has_next() || !color_camera_info_reader_.has_next() ||
    !tf_reader_.has_next())
  {
    if (!depth_reader_.has_next()) {
      LOG(WARNING) << "No mesages on the depth topic: " << depth_topic;
    }
    if (!depth_camera_info_reader_.has_next()) {
      LOG(WARNING) << "No mesages on the depth camera info topic: " << depth_camera_info_topic;
    }
    if (!color_reader_.has_next()) {
      LOG(WARNING) << "No mesages on the color topic: " << color_topic;
    }
    if (!color_camera_info_reader_.has_next()) {
      LOG(WARNING) << "No mesages on the color camera info topic: " << color_camera_info_topic;
    }
    if (!tf_reader_.has_next()) {
      LOG(WARNING) << "No mesages on the tf topic: /tf";
    }
    setup_success_ = false;
  }
}

datasets::DataLoadResult RosDataLoader::loadNext(
  DepthImage * depth_frame_ptr,                                                // NOLINT
  Transform * T_L_C_ptr,                                                       // NOLINT
  Camera * camera_ptr,                                                         // NOLINT
  ColorImage * color_frame_ptr)
{
  // Load two cams, and two transforms
  Transform T_L_color_ptr;
  Camera color_camera_ptr;
  auto load_result = loadNext(
    depth_frame_ptr, T_L_C_ptr, camera_ptr, color_frame_ptr,
    &T_L_color_ptr, &color_camera_ptr);
  CHECK(areCamerasEqual(*camera_ptr, color_camera_ptr, *T_L_C_ptr, T_L_color_ptr))
    << "You tried to call the loadNext() function which assumes depth and "
    "color cameras are the same, but they're different.";
  return load_result;
}

DataLoadResult RosDataLoader::loadNext(
  DepthImage * depth_frame_ptr,                                      // NOLINT
  Transform * T_L_D_ptr,                                             // NOLINT
  Camera * depth_camera_ptr,                                         // NOLINT
  ColorImage * color_frame_ptr,                                      // NOLINT
  Transform * T_L_C_ptr,                                             // NOLINT
  Camera * color_camera_ptr)
{
  CHECK(setup_success_) << "The RosDataLoader did not construct in a valid state. Likely missing "
    "messages on (at least) one topic.";
  CHECK_NOTNULL(depth_frame_ptr);
  CHECK_NOTNULL(T_L_D_ptr);
  CHECK_NOTNULL(depth_camera_ptr);
  CHECK_NOTNULL(color_frame_ptr);
  CHECK_NOTNULL(T_L_C_ptr);
  CHECK_NOTNULL(color_camera_ptr);
  // Step forward on all topics until we get a match (or the bag is over).
  if (!stepUntilNextMatchingMessages()) {
    return datasets::DataLoadResult::kNoMoreData;
  }
  CHECK(depth_image_msg_.has_value());
  CHECK(depth_camera_info_msg_.has_value());
  CHECK(color_image_msg_.has_value());
  CHECK(color_camera_info_msg_.has_value());
  CHECK(tf_msg_.has_value());
  // Convert to nvblox types.
  *depth_camera_ptr = conversions::cameraFromMessage(depth_camera_info_msg_.value());
  *color_camera_ptr = conversions::cameraFromMessage(color_camera_info_msg_.value());
  bool success = conversions::depthImageFromRosMessageAsync(
    depth_image_msg_.value(), depth_frame_ptr, &depth_image_conversion_scratch_, ros_logger_,
    *cuda_stream_);
  CHECK(success) << "Depth image conversion failed";
  success = conversions::colorImageFromImageMessageAsync(
    color_image_msg_.value(), color_frame_ptr, &rgb_image_conversion_scratch_,
    &bgra_image_conversion_scratch_, ros_logger_, *cuda_stream_);
  CHECK(success) << "Color image conversion failed";

  // Retrieve the transform from the camera frame (found in the image
  // header), to the global/layer frame, specified as a member variable.
  // NOTE(alexmillane): It's possible that the transform in some datasets goes in and
  // out of availability and therefore we return datasets::DataLoadResult::kBadData
  // and move to the next frame.
  auto stamp = (*depth_image_msg_).header.stamp;
  success = getTransformAtTime(
    global_frame_id_, (*depth_image_msg_).header.frame_id, stamp,
    tf_buffer_, T_L_D_ptr);
  if (!success) {
    LOG(WARNING) << "Couldn't find transform from: " << (*depth_image_msg_).header.frame_id
                 << " to " << global_frame_id_ << ". Moving to next messages.";
    return datasets::DataLoadResult::kBadFrame;
  }
  stamp = (*color_image_msg_).header.stamp;
  success = getTransformAtTime(
    global_frame_id_, (*color_image_msg_).header.frame_id, stamp,
    tf_buffer_, T_L_C_ptr);
  if (!success) {
    LOG(WARNING) << "Couldn't find transform from: " << (*color_image_msg_).header.frame_id
                 << " to " << global_frame_id_ << ". Moving to next messages.";
    return datasets::DataLoadResult::kBadFrame;
  }

  return datasets::DataLoadResult::kSuccess;
}

bool RosDataLoader::isMatch(
  const sensor_msgs::msg::Image & depth_msg,                                            // NOLINT
  const sensor_msgs::msg::CameraInfo & depth_camera_info_msg,                           // NOLINT
  const sensor_msgs::msg::Image & color_msg,                                            // NOLINT
  const sensor_msgs::msg::CameraInfo & color_camera_info_msg,                           // NOLINT
  const tf2_msgs::msg::TFMessage & tf_msg)
{
  // Match conditions (both must be true):
  // 1) Depth, depth camera_info, color, and color camera_info have an exact
  // timestamp match.
  // 2) The stamp of the last loaded tf message is greater than the depth
  // stamp + the tf_lead_time.
  // NOTE(alexmillane): The requirement for exact timestamp matching is brittle.
  // It works for perceptor where depth and color are computed from the same image
  // but it wont work for realsense, where color and depth are different cameras.
  // TODO(alexmillane): Upgrade to approximate timestamp matching.
  return (depth_msg.header.stamp == depth_camera_info_msg.header.stamp) &&
         (color_msg.header.stamp == color_camera_info_msg.header.stamp) &&
         (depth_msg.header.stamp == color_msg.header.stamp) &&
         (depth_msg.header.stamp <= getEarlierTime(getLowestStamp(tf_msg), tf_lead_time_s_));
}

bool RosDataLoader::stepUntilNextMatchingMessages()
{
  // If not yet started load an image on each channel.
  if (!depth_image_msg_.has_value() || !depth_camera_info_msg_.has_value() ||
    !color_image_msg_.has_value() || !color_camera_info_msg_.has_value() ||
    !tf_msg_.has_value())
  {
    // Initialize messages
    depth_image_msg_ = sensor_msgs::msg::Image();
    depth_camera_info_msg_ = sensor_msgs::msg::CameraInfo();
    color_image_msg_ = sensor_msgs::msg::Image();
    color_camera_info_msg_ = sensor_msgs::msg::CameraInfo();
    tf_msg_ = tf2_msgs::msg::TFMessage();
    // Step once on each channel to get started.
    CHECK(stepDepthMsg(&depth_image_msg_.value())) << "Failed to step first message.";
    CHECK(stepDepthCameraInfoMsg(&depth_camera_info_msg_.value()))
      << "Failed to step first message.";
    CHECK(stepColorMsg(&color_image_msg_.value())) << "Failed to step first message.";
    CHECK(stepColorCameraInfoMsg(&color_camera_info_msg_.value()))
      << "Failed to step first message.";
    CHECK(stepTfMsg(&tf_msg_.value())) << "Failed to step first message.";
  } else {
    // Step one "random" image forward. Next we'll step the others until they
    // catch up
    // NOTE(alexmillane): When we enter this function, the image streams already have a match
    // from the last time. This stepping of one of the image channels breaks the match. We then
    // enter the loop that loops until we have a match again.
    if (!stepDepthMsg(&depth_image_msg_.value())) {
      return false;
    }
  }
  CHECK(depth_image_msg_.has_value());
  CHECK(depth_camera_info_msg_.has_value());
  CHECK(color_image_msg_.has_value());
  CHECK(color_camera_info_msg_.has_value());
  CHECK(tf_msg_.has_value());

  // Loop, stepping until we reach a match or we run out of messages.
  std::vector<std::function<bool(void)>> step_functions = {
    std::bind(&RosDataLoader::stepDepthMsg, this, &depth_image_msg_.value()),
    std::bind(&RosDataLoader::stepDepthCameraInfoMsg, this, &depth_camera_info_msg_.value()),
    std::bind(&RosDataLoader::stepColorMsg, this, &color_image_msg_.value()),
    std::bind(&RosDataLoader::stepColorCameraInfoMsg, this, &color_camera_info_msg_.value()),
    std::bind(&RosDataLoader::stepTfMsg, this, &tf_msg_.value()),
  };
  std::vector<std::string> debug_strings = {"depth",              // NOLINT
    "color_camera_info",                                          // NOLINT
    "color",                                                      // NOLINT
    "color_camera_info",                                          // NOLINT
    "tf"};
  while (messagesAvailable(
      depth_reader_, color_reader_, depth_camera_info_reader_,
      color_camera_info_reader_, tf_reader_) &&
    !isMatch(
      depth_image_msg_.value(), depth_camera_info_msg_.value(),
      color_image_msg_.value(), color_camera_info_msg_.value(), tf_msg_.value()))
  {
    DLOG(INFO) << "No match found: \n\t" << toNanoSeconds((*depth_image_msg_).header.stamp)
               << "\n\t" << toNanoSeconds((*depth_camera_info_msg_).header.stamp) << "\n\t"
               << toNanoSeconds((*color_image_msg_).header.stamp) << "\n\t"
               << toNanoSeconds((*color_camera_info_msg_).header.stamp) << "\n\t"
               << toNanoSeconds(getLowestStamp(*tf_msg_));
    // Search for lowest stamp and step the message
    // NOTE(alexmillane): The order of these stamps has to match the order of
    // the step functions.
    const std::vector<builtin_interfaces::msg::Time> stamps = {
      (*depth_image_msg_).header.stamp, (*depth_camera_info_msg_).header.stamp,
      (*color_image_msg_).header.stamp, (*color_camera_info_msg_).header.stamp,
      getEarlierTime(getLowestStamp(*tf_msg_), tf_lead_time_s_)};
    const int min_idx =
      std::min_element(stamps.begin(), stamps.end(), RosTimeLessThan()) - stamps.begin();
    DLOG(INFO) << "minimum idx: " << min_idx;
    DLOG(INFO) << "minimum message: " << debug_strings[min_idx];
    // Step
    step_functions[min_idx]();
  }

  // If we terminated the while loop at a match, print some debug and return.
  if (isMatch(
      depth_image_msg_.value(), depth_camera_info_msg_.value(), color_image_msg_.value(),
      color_camera_info_msg_.value(), tf_msg_.value()))
  {
    DLOG(INFO) << "Match found: \n\t" << toNanoSeconds((*depth_image_msg_).header.stamp) << "\n\t"
               << toNanoSeconds((*depth_camera_info_msg_).header.stamp) << "\n\t"
               << toNanoSeconds((*color_image_msg_).header.stamp) << "\n\t"
               << toNanoSeconds((*color_camera_info_msg_).header.stamp) << "\n\t"
               << toNanoSeconds(getLowestStamp(*tf_msg_));
  } else {
    // We terminated before a match. Retrieval failed.
    LOG(INFO) << "Ran out of data looking for a match.";
    return false;
  }
  return true;
}

bool RosDataLoader::stepDepthMsg(sensor_msgs::msg::Image * msg_ptr)
{
  CHECK_NOTNULL(msg_ptr);
  return stepNextMessage<sensor_msgs::msg::Image>(msg_ptr, &depth_reader_, "depth");
}

bool RosDataLoader::stepDepthCameraInfoMsg(sensor_msgs::msg::CameraInfo * msg_ptr)
{
  CHECK_NOTNULL(msg_ptr);
  return stepNextMessage<sensor_msgs::msg::CameraInfo>(
    msg_ptr, &depth_camera_info_reader_,
    "depth_info");
}

bool RosDataLoader::stepColorMsg(sensor_msgs::msg::Image * msg_ptr)
{
  CHECK_NOTNULL(msg_ptr);
  return stepNextMessage<sensor_msgs::msg::Image>(msg_ptr, &color_reader_, "color");
}

bool RosDataLoader::stepColorCameraInfoMsg(sensor_msgs::msg::CameraInfo * msg_ptr)
{
  CHECK_NOTNULL(msg_ptr);
  return stepNextMessage<sensor_msgs::msg::CameraInfo>(
    msg_ptr, &color_camera_info_reader_,
    "color_info");
}

bool RosDataLoader::stepTfMsg(tf2_msgs::msg::TFMessage * msg_ptr)
{
  CHECK_NOTNULL(msg_ptr);
  const bool result = stepNextMessage<tf2_msgs::msg::TFMessage>(msg_ptr, &tf_reader_, "tf");
  // We additionally load the tfs into the buffer.
  for (const auto & transform : msg_ptr->transforms) {
    tf_buffer_.setTransform(transform, "default_authority");
  }
  return result;
}

}  // namespace ros
}  // namespace datasets
}  // namespace nvblox
