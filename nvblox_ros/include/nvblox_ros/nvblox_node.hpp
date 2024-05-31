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

#ifndef NVBLOX_ROS__NVBLOX_NODE_HPP_
#define NVBLOX_ROS__NVBLOX_NODE_HPP_

#include <nvblox/nvblox.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <chrono>
#include <list>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

#include <libstatistics_collector/topic_statistics_collector/topic_statistics_collector.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <nvblox_msgs/srv/file_path.hpp>
#include <nvblox_msgs/srv/esdf_and_gradients.hpp>

#include "nvblox_ros/conversions/image_conversions.hpp"
#include "nvblox_ros/conversions/layer_conversions.hpp"
#include "nvblox_ros/conversions/mesh_conversions.hpp"
#include "nvblox_ros/conversions/pointcloud_conversions.hpp"
#include "nvblox_ros/conversions/esdf_slice_conversions.hpp"
#include "nvblox_ros/conversions/esdf_and_gradients_conversions.hpp"
#include "nvblox_ros/mapper_initialization.hpp"
#include "nvblox_ros/transformer.hpp"
#include "nvblox_ros/camera_cache.hpp"
#include "nvblox_ros/nitros_types.hpp"
#include "nvblox_ros/nvblox_node_params.hpp"

namespace nvblox
{

class NvbloxNode : public rclcpp::Node
{
public:
  explicit NvbloxNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "nvblox_node");
  virtual ~NvbloxNode();

  // Setup. These are called by the constructor.
  void initializeMultiMapper();
  void subscribeToTopics();
  void advertiseTopics();
  void advertiseServices();
  void setupTimers();

  // Callback functions. These just stick images in a queue.
  void depthImageCallback(const NitrosView & image_view);
  void colorImageCallback(const NitrosView & image_view);
  void pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud);
  void savePly(
    const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response);
  void saveMap(
    const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response);
  void loadMap(
    const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response);
  void saveRates(
    const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response);
  void saveTimings(
    const std::shared_ptr<nvblox_msgs::srv::FilePath::Request> request,
    std::shared_ptr<nvblox_msgs::srv::FilePath::Response> response);
  void getEsdfAndGradientService(
    const std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Request> request,
    std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Response> response);

  // Main tick function that process all input data queues in order
  virtual void tick();

  // Publish data on fixed frequency
  void publishLayers();

  // Process data
  virtual bool processDepthImage(const NitrosViewPtrAndFrameId & depth_image_view);
  virtual bool processColorImage(const NitrosViewPtrAndFrameId & color_image_view);

  virtual bool processLidarPointcloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_ptr);

  // Return true if the tf-tree contains a transform for the given frame_id and timestamp
  bool canTransform(const std::string & frame_id, const rclcpp::Time & timestamp);

  void publishSlicePlane(const rclcpp::Time & timestamp, const Transform & T_L_C);

  // Decay the dynamic occupancy and static tsdf grid on fixed frequency
  void decayDynamicOccupancy();
  void decayTsdf();

protected:
  // Process functions for individual queues
  virtual void processDepthQueue();
  virtual void processColorQueue();
  virtual void processPointcloudQueue();
  virtual void processEsdf();
  virtual void processMesh();

  // Return true if the time between the two passed timestamps is sufficient to trigger an action
  // under the requested rate.
  bool shouldProcess(
    const rclcpp::Time & time_now, const rclcpp::Time & time_last,
    const float desired_frequency_hz);

  // Publish the dynamic outputs
  void publishDynamics(
    const std::string & camera_frame_id);

  // Publish the back projected depth image for debug purposes
  void publishBackProjectedDepth(const Camera & camera, const Transform & T_L_C);

  // Helper function to update the esdf of a specific mapper
  void updateEsdf(
    const std::string & name,
    const std::shared_ptr<Mapper> & mapper);

  // Helper function to slice and publish the esdf of a specific mapper
  void sliceAndPublishEsdf(
    const std::string & name,
    const std::shared_ptr<Mapper> & mapper,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pointcloud_publisher,
    const rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr & slice_publisher,
    const Mapper * mapper_2 = nullptr);

  // Map clearing
  void clearMapOutsideOfRadiusOfLastKnownPose();

  /// Used by callbacks (internally) to add messages to queues.
  /// @tparam MessageType The type of the Message stored by the queue.
  /// @param queue_name Name of the queue, used for logging.
  /// @param message Message to be added to the queue.
  /// @param queue_ptr Queue where to add the message.
  /// @param queue_mutex_ptr Mutex protecting the queue.
  template<typename MessageType>
  void pushMessageOntoQueue(
    const std::string & queue_name,
    MessageType message,
    std::list<MessageType> * queue_ptr,
    std::mutex * queue_mutex_ptr);
  template<typename MessageType>
  void printMessageArrivalStatistics(
    const MessageType & message, const std::string & output_prefix,
    libstatistics_collector::topic_statistics_collector::
    ReceivedMessagePeriodCollector<MessageType> * statistics_collector);

  // Used internally to unify processing of queues that process a message and a
  // matching transform.
  template<typename MessageType>
  using ProcessMessageCallback = std::function<bool (const MessageType &)>;
  template<typename MessageType>
  using MessageReadyCallback = std::function<bool (const MessageType &)>;

  /// Processes a queue of messages by detecting if they're ready and then
  /// passing them to a callback.
  /// @tparam MessageType The type of the messages in the queue.
  /// @param queue_ptr Queue of messages to process.
  /// @param queue_mutex_ptr Mutex protecting the queue.
  /// @param message_ready_check Callback called on each message to check if
  /// it's ready to be processed
  /// @param callback Callback to process each ready message.
  template<typename MessageType>
  void processMessageQueue(
    std::list<MessageType> * queue_ptr, std::mutex * queue_mutex_ptr,
    MessageReadyCallback<MessageType> message_ready_check,
    ProcessMessageCallback<MessageType> callback);

  // Declares a ROS parameter.
  // Calls the underlying method from rclcpp, but also adds the parameter and value to
  // parameter tree for printing.
  template<typename ParamType>
  ParamType declareParameter(
    const std::string & name, const ParamType & default_value,
    nvblox::parameters::ParameterTreeNode * parameter_tree);

  // ROS publishers and subscribers

  // Transformer to handle... everything, let's be honest.
  Transformer transformer_;

  // Throttling debug messages to reduce spamming the console
  static constexpr float kTimeBetweenDebugMessagesMs = 1000.0;

  /// Number of cameras supported (number of subscribers created).
  static constexpr size_t kMaxNumCameras = 4;

  /// Depth topics to listen to. At least one of these topics has to transmit images. All topics
  /// are added to the same processing queue (depth or color) and thus gets the same treatment.
  static constexpr std::array<const char *,
    kMaxNumCameras> kDepthTopicBaseNames =
  {
    // Multi-camera topics
    "camera_0/depth",
    "camera_1/depth",
    "camera_2/depth",
    "camera_3/depth",
  };

  /// Color topics to listen to.
  static constexpr std::array<const char *,
    kMaxNumCameras> kColorTopicBaseNames =
  {
    // Multi-camera topics
    "camera_0/color",
    "camera_1/color",
    "camera_2/color",
    "camera_3/color",
  };


  /// Image + info subscribers
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;
  std::vector<std::shared_ptr<NitrosViewSubscriber>> nitros_image_subs_;

  // Pointcloud sub.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    pointcloud_sub_;

  // Optional transform subs.
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
    transform_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // Publishers
  rclcpp::Publisher<nvblox_msgs::msg::Mesh>::SharedPtr mesh_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    static_esdf_pointcloud_publisher_;
  rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr
    static_map_slice_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    slice_bounds_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    mesh_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    tsdf_layer_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    occupancy_layer_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    color_layer_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    freespace_layer_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    dynamic_occupancy_layer_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    back_projected_depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    dynamic_points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    dynamic_depth_frame_overlay_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    dynamic_esdf_pointcloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    combined_esdf_pointcloud_publisher_;
  rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr
    dynamic_map_slice_publisher_;
  rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr
    combined_map_slice_publisher_;

  // Services.
  rclcpp::Service<nvblox_msgs::srv::FilePath>::SharedPtr save_ply_service_;
  rclcpp::Service<nvblox_msgs::srv::FilePath>::SharedPtr save_map_service_;
  rclcpp::Service<nvblox_msgs::srv::FilePath>::SharedPtr load_map_service_;
  rclcpp::Service<nvblox_msgs::srv::FilePath>::SharedPtr save_rates_service_;
  rclcpp::Service<nvblox_msgs::srv::FilePath>::SharedPtr save_timings_service_;
  rclcpp::Service<nvblox_msgs::srv::EsdfAndGradients>::SharedPtr send_esdf_and_gradient_service_;

  // Callback groups.
  rclcpp::CallbackGroup::SharedPtr group_processing_;

  // Timers.
  rclcpp::TimerBase::SharedPtr queue_processing_timer_;

  // ROS & nvblox settings
  MultiMapper::Params multi_mapper_params_;

  // Collection of params for the nvblox node
  NvbloxNodeParams params_;

  // Counter for back projection subsampling
  uint32_t back_projection_idx_ = 0;

  /// The last time processing occurred. Used to maintain rates.
  /// NOTE: Because we support multiple cameras, the first two maps map from the frame_id
  /// of the camera to the timestamp of the last image integrated from that camera.
  std::unordered_map<std::string, rclcpp::Time> integrate_depth_last_times_;
  std::unordered_map<std::string, rclcpp::Time> integrate_color_last_times_;
  rclcpp::Time integrate_lidar_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time update_mesh_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time update_esdf_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time publish_layer_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time decay_tsdf_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time decay_dynamic_occupancy_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time clear_map_outside_radius_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  /// The time stamp of the last frame contributing to the reconstruction.
  rclcpp::Time newest_integrated_depth_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // The QoS settings for the image input topics
  const std::string kDefaultInputQos_ = "SYSTEM_DEFAULT";

  // MultiMapper
  // Holding the static and dynamic mapper and
  // handling where input data gets integrated.
  std::shared_ptr<MultiMapper> multi_mapper_;

  // Direct access to Mappers
  // Holding the map layers and their associated integrators
  // - TsdfLayer, FreespaceLayer, ColorLayer, EsdfLayer, MeshLayer
  std::shared_ptr<Mapper> static_mapper_;
  std::shared_ptr<Mapper> dynamic_mapper_;

  // Various converters for ROS message generation.
  conversions::LayerConverter layer_converter_;
  conversions::PointcloudConverter pointcloud_converter_;
  conversions::EsdfSliceConverter esdf_slice_converter_;
  conversions::EsdfAndGradientsConverter esdf_and_gradients_converter_;

  // Caches for GPU images
  ColorImage color_image_{MemoryType::kDevice};
  DepthImage depth_image_{MemoryType::kDevice};
  DepthImage pointcloud_image_{MemoryType::kDevice};
  Image<conversions::Rgb> rgb_image_tmp_{MemoryType::kDevice};

  // Object for back projecting image to a pointcloud.
  DepthImageBackProjector image_back_projector_;

  // Message statistics (useful for debugging)
  libstatistics_collector::topic_statistics_collector::
  ReceivedMessagePeriodCollector<NitrosView> depth_frame_statistics_;
  libstatistics_collector::topic_statistics_collector::
  ReceivedMessagePeriodCollector<NitrosView> rgb_frame_statistics_;
  libstatistics_collector::topic_statistics_collector::
  ReceivedMessagePeriodCollector<sensor_msgs::msg::PointCloud2>
  pointcloud_frame_statistics_;

  // Cache the last known number of subscribers.
  size_t mesh_subscriber_count_ = 0;

  // Sensor data queues.
  std::list<NitrosViewPtrAndFrameId> depth_image_queue_;
  std::list<NitrosViewPtrAndFrameId> color_image_queue_;

  std::list<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pointcloud_queue_;

  // Image queue mutexes.
  std::mutex depth_queue_mutex_;
  std::mutex color_queue_mutex_;
  std::mutex pointcloud_queue_mutex_;

  // Counts the number of messages dropped from image queues.
  // Maps the sensor data queue to the number of messages that have been dropped.
  std::unordered_map<std::string, int> number_of_dropped_messages_;

  // Device caches
  Pointcloud pointcloud_C_device_;
  Pointcloud pointcloud_L_device_;

  // The idle timer measures time spent *outside* the main tick function and can thus be used to
  // monitor how much headroom there is until the system gets saturated. A lower number <=1ms/tick
  // means that the system is under high pressure.
  static constexpr bool kStartStopped = true;
  timing::Timer idle_timer_{"ros/idle", kStartStopped};

  // Cuda stream for GPU work
  nvblox::CudaStreamOwning cuda_stream_;

  // Cached intrinsic calibration for cameras. Separate caches for depth and color in order to
  // support different intrinsics between the two.
  CameraCache depth_camera_cache_;
  CameraCache color_camera_cache_;

  // Parameter tree for parameter printing.
  nvblox::parameters::ParameterTreeNode parameter_tree_{"nvblox_node", {}};
};

}  // namespace nvblox

#include "nvblox_ros/impl/nvblox_node_impl.hpp"

#endif  // NVBLOX_ROS__NVBLOX_NODE_HPP_
