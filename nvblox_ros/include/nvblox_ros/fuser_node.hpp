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

#ifndef NVBLOX_ROS__FUSER_NODE_HPP_
#define NVBLOX_ROS__FUSER_NODE_HPP_

#include <nvblox/nvblox.h>

#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "nvblox/datasets/3dmatch.h"
#include "nvblox/datasets/redwood.h"
#include "nvblox/datasets/replica.h"

#include "nvblox_ros/conversions/esdf_slice_conversions.hpp"
#include "nvblox_ros/conversions/image_conversions.hpp"
#include "nvblox_ros/conversions/mesh_conversions.hpp"
#include "nvblox_ros/conversions/pointcloud_conversions.hpp"
#include "nvblox_ros/conversions/transform_conversions.hpp"
#include "nvblox_ros/layer_publishing.hpp"
#include "nvblox_ros/mapper_initialization.hpp"
#include "nvblox_ros/node_params.hpp"
#include "nvblox_ros/terminal_reading.hpp"

namespace nvblox
{

class FuserNode : public rclcpp::Node
{
public:
  /// The state of the fuser when in continuous (rather than interactive) mode.
  enum class ContinuousModeState { kRunning, kPaused };

  explicit FuserNode(
    const std::string & node_name = "fuser_node",
    std::shared_ptr<CudaStream> cuda_stream =
    std::make_shared<CudaStreamOwning>());
  virtual ~FuserNode() = default;

  /// @brief If update_on_key is true, we fuse a new frame if a space key is
  // detected. Otherwise we just fuse the next frame.
  /// @return true as long as integrating frames is successful (false if no
  ///  frames left).
  bool update();

private:
  /// @brief Function to fuse the next frame and publish the resulting
  ///  reconstruction.
  /// @return true as long as integrating frames is successful (false if no
  ///  frames left).
  bool fuseNextFrame();

  /// @brief Reads keys from terminal and integrates the next frame when space
  ///  key is detected.
  /// @return true as long as integrating frames is
  ///  successful (false if no / frames left).
  bool updateOnKey();

  /// @brief Integrates the next frame if the system isn't paused. Also reads
  /// the keyboard in order to update the pause status.
  /// @return True if we haven't run out of data.
  bool updateIfNotPaused();

  /// @brief Getter for fuser node params.
  /// @return The fuser node params.
  const FuserNodeParams & params() {return params_;}

  /// @brief Function for printing nvblox statistics to console.
  void printStatistics();

  // Fuser for integrating frames
  std::unique_ptr<Fuser> fuser_;

  // Direct access to underlying mapper object
  std::shared_ptr<Mapper> mapper_;

  // Parameter handling
  FuserNodeParams params_;
  nvblox::parameters::ParameterTreeNode parameter_tree_{"fuser_node", {}};

  // The state of the mapper when in continuous mode.
  ContinuousModeState continuous_mode_state_ = ContinuousModeState::kRunning;

  // ROS publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_frame_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_frame_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    esdf_pointcloud_publisher_;
  rclcpp::Publisher<nvblox_msgs::msg::Mesh>::SharedPtr mesh_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    back_projected_depth_publisher_;
  std::unique_ptr<LayerPublisher> layer_publisher_;

  // Broadcasting tf2 transforms
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Object for back projecting image to a pointcloud.
  DepthImageBackProjector image_back_projector_;

  // Conversion helpers
  conversions::EsdfSliceConverter esdf_slice_converter_;
  conversions::PointcloudConverter pointcloud_converter_;

  // Tracking of frame number
  int current_frame_number_ = 0;

  // Device caches
  Pointcloud pointcloud_C_device_;
  Pointcloud pointcloud_L_device_;

  // Cuda stream for GPU work
  std::shared_ptr<CudaStream> cuda_stream_ = nullptr;
};

}  // namespace nvblox

#endif  // NVBLOX_ROS__FUSER_NODE_HPP_
