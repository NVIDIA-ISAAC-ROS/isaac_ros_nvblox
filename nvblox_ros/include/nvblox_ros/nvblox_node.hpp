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

#ifndef NVBLOX_ROS__NVBLOX_NODE_HPP_
#define NVBLOX_ROS__NVBLOX_NODE_HPP_

#include <nvblox/nvblox.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <libstatistics_collector/topic_statistics_collector/topic_statistics_collector.hpp>
#include <nvblox_msgs/srv/file_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "nvblox_ros/conversions/image_conversions.hpp"
#include "nvblox_ros/conversions/layer_conversions.hpp"
#include "nvblox_ros/conversions/mesh_conversions.hpp"
#include "nvblox_ros/conversions/pointcloud_conversions.hpp"
#include "nvblox_ros/conversions/esdf_slice_conversions.hpp"
#include "nvblox_ros/mapper_initialization.hpp"
#include "nvblox_ros/transformer.hpp"

namespace nvblox
{

class NvbloxNode : public rclcpp::Node
{
public:
  explicit NvbloxNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "nvblox_node");
  virtual ~NvbloxNode() = default;

  // Setup. These are called by the constructor.
  void getParameters();
  void initializeMultiMapper();
  void subscribeToTopics();
  void advertiseTopics();
  void advertiseServices();
  void setupTimers();

  // Callback functions. These just stick images in a queue.
  void depthImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_img_ptr,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);
  void colorImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color_img_ptr,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & color_info_msg);
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

  // Does whatever processing there is to be done, depending on what
  // transforms are available.
  virtual void processDepthQueue();
  virtual void processColorQueue();
  virtual void processPointcloudQueue();
  virtual void processEsdf();
  virtual void processMesh();

  // Publish data on fixed frequency
  void publishOccupancyPointcloud();

  // Process data
  virtual bool processDepthImage(
    const std::pair<sensor_msgs::msg::Image::ConstSharedPtr,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr> &
    depth_camera_pair);
  virtual bool processColorImage(
    const std::pair<sensor_msgs::msg::Image::ConstSharedPtr,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr> &
    color_camera_pair);
  virtual bool processLidarPointcloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_ptr);

  bool canTransform(const std_msgs::msg::Header & header);

  void publishSlicePlane(const rclcpp::Time & timestamp, const Transform & T_L_C);

  // Decay the dynamic occupancy grid on fixed frequency
  void decayDynamicOccupancy();

protected:
  // Publish the freespace layer
  void publishFreespace(
    const Transform & T_L_C);

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
  /// @param message Message to be added to the queue.
  /// @param queue_ptr Queue where to add the message.
  /// @param queue_mutex_ptr Mutex protecting the queue.
  template<typename MessageType>
  void pushMessageOntoQueue(
    MessageType message,
    std::deque<MessageType> * queue_ptr,
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
    std::deque<MessageType> * queue_ptr, std::mutex * queue_mutex_ptr,
    MessageReadyCallback<MessageType> message_ready_check,
    ProcessMessageCallback<MessageType> callback);

  // Check if interval between current stamp
  bool isUpdateTooFrequent(
    const rclcpp::Time & current_stamp,
    const rclcpp::Time & last_update_stamp,
    float max_update_rate_hz);

  template<typename MessageType>
  void limitQueueSizeByDeletingOldestMessages(
    const int max_num_messages, const std::string & queue_name,
    std::deque<MessageType> * queue_ptr, std::mutex * queue_mutex_ptr);

  // ROS publishers and subscribers

  // Transformer to handle... everything, let's be honest.
  Transformer transformer_;

  // Time Sync
  typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>
    time_policy_t;

  // Depth sub.
  std::shared_ptr<message_filters::Synchronizer<time_policy_t>> timesync_depth_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo>
  depth_camera_info_sub_;

  // Color sub
  std::shared_ptr<message_filters::Synchronizer<time_policy_t>> timesync_color_;
  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo>
  color_camera_info_sub_;

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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    static_occupancy_publisher_;
  rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr
    static_map_slice_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    slice_bounds_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    mesh_marker_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    back_projected_depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    dynamic_points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    dynamic_depth_frame_overlay_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr freespace_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    dynamic_occupancy_publisher_;
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

  // Callback groups.
  rclcpp::CallbackGroup::SharedPtr group_processing_;

  // Timers.
  rclcpp::TimerBase::SharedPtr depth_processing_timer_;
  rclcpp::TimerBase::SharedPtr color_processing_timer_;
  rclcpp::TimerBase::SharedPtr pointcloud_processing_timer_;
  rclcpp::TimerBase::SharedPtr occupancy_publishing_timer_;
  rclcpp::TimerBase::SharedPtr esdf_processing_timer_;
  rclcpp::TimerBase::SharedPtr mesh_processing_timer_;
  rclcpp::TimerBase::SharedPtr clear_outside_radius_timer_;
  rclcpp::TimerBase::SharedPtr dynamic_occupancy_decay_timer_;

  // ROS & nvblox settings
  float voxel_size_ = 0.05f;
  EsdfMode esdf_mode_ = EsdfMode::k2D;
  bool publish_esdf_distance_slice_ = true;
  MappingType mapping_type_ = MappingType::kStaticTsdf;
  MultiMapper::Params multi_mapper_params_;

  // Toggle parameters
  bool use_depth_ = true;
  bool use_lidar_ = true;
  bool use_color_ = true;
  bool compute_esdf_ = true;
  bool compute_mesh_ = true;

  // LIDAR settings
  // Defaults for Velodyne VLP16
  int lidar_width_ = 1800;
  int lidar_height_ = 16;
  float lidar_vertical_fov_rad_ = 30.0 * M_PI / 180.0;

  // Alternate LiDAR Settings for lidars which are not even around 0 elevation
  // These settings are alternate to specifying the vertical FoV above.
  // The node will use these settings if "lidar_non_equal_vertical_fov" is set
  // to true. Defaults for Hesai PandarXT32
  bool use_non_equal_vertical_fov_lidar_params_ = false;
  float min_angle_below_zero_elevation_rad_ = 20.0 * M_PI / 180.0;
  float max_angle_above_zero_elevation_rad_ = 15.0 * M_PI / 180.0;

  // Slice visualization params
  std::string slice_visualization_attachment_frame_id_ = "base_link";
  float slice_visualization_side_length_ = 10.0f;

  // ROS settings & update throttles
  std::string global_frame_ = "odom";
  /// Pose frame to use if using transform topics.
  std::string pose_frame_ = "base_link";
  float max_depth_update_hz_ = 30.0f;
  float max_color_update_hz_ = 5.0f;
  float max_lidar_update_hz_ = 10.0f;
  float mesh_update_rate_hz_ = 5.0f;
  float esdf_update_rate_hz_ = 2.0f;
  float static_occupancy_publication_rate_hz_ = 2.0f;

  float dynamic_occupancy_decay_rate_hz_ = 10.0f;

  /// Specifies what rate to poll the color & depth updates at.
  /// Will exit as no-op if no new images are in the queue so it is safe to
  /// set this higher than you expect images to come in at.
  float max_poll_rate_hz_ = 100.0f;

  /// How many messages to store in the sensor messages queues (depth, color,
  /// lidar) before deleting oldest messages.
  int maximum_sensor_message_queue_length_ = 30;

  /// Map clearing params
  /// Note that values <=0.0 indicate that no clearing is performed.
  float map_clearing_radius_m_ = -1.0f;
  std::string map_clearing_frame_id_ = "base_link";
  float clear_outside_radius_rate_hz_ = 1.0f;

  // The QoS settings for the image input topics
  std::string depth_qos_str_ = "SYSTEM_DEFAULT";
  std::string color_qos_str_ = "SYSTEM_DEFAULT";
  std::string pointcloud_qos_str_ = "SYSTEM_DEFAULT";


  // MultiMapper
  // Holding the static and dynamic mapper and
  // handling where input data gets integrated.
  std::shared_ptr<MultiMapper> multi_mapper_;

  // Direct access to Mappers
  // Holding the map layers and their associated integrators
  // - TsdfLayer, FreespaceLayer, ColorLayer, EsdfLayer, MeshLayer
  std::shared_ptr<Mapper> static_mapper_;
  std::shared_ptr<Mapper> dynamic_mapper_;

  // The most important part: the ROS converter. Just holds buffers as state.
  conversions::LayerConverter layer_converter_;
  conversions::PointcloudConverter pointcloud_converter_;
  conversions::EsdfSliceConverter esdf_slice_converter_;

  // Caches for GPU images
  ColorImage color_image_{MemoryType::kDevice};
  DepthImage depth_image_{MemoryType::kDevice};
  DepthImage pointcloud_image_{MemoryType::kDevice};

  // Object for back projecting image to a pointcloud.
  DepthImageBackProjector image_back_projector_;

  // Message statistics (useful for debugging)
  libstatistics_collector::topic_statistics_collector::
  ReceivedMessagePeriodCollector<sensor_msgs::msg::Image>
  depth_frame_statistics_;
  libstatistics_collector::topic_statistics_collector::
  ReceivedMessagePeriodCollector<sensor_msgs::msg::Image>
  rgb_frame_statistics_;
  libstatistics_collector::topic_statistics_collector::
  ReceivedMessagePeriodCollector<sensor_msgs::msg::PointCloud2>
  pointcloud_frame_statistics_;

  // State for integrators running at various speeds.
  rclcpp::Time last_depth_update_time_;
  rclcpp::Time last_color_update_time_;
  rclcpp::Time last_lidar_update_time_;

  // Cache the last known number of subscribers.
  size_t mesh_subscriber_count_ = 0;

  // Image queues.
  std::deque<std::pair<sensor_msgs::msg::Image::ConstSharedPtr,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr>>
  depth_image_queue_;
  std::deque<std::pair<sensor_msgs::msg::Image::ConstSharedPtr,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr>>
  color_image_queue_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pointcloud_queue_;

  // Image queue mutexes.
  std::mutex depth_queue_mutex_;
  std::mutex color_queue_mutex_;
  std::mutex pointcloud_queue_mutex_;

  // Keeps track of the mesh blocks deleted such that we can publish them for
  // deletion in the rviz plugin
  Index3DSet mesh_blocks_deleted_;

  // Device caches
  Pointcloud pointcloud_C_device_;
  Pointcloud pointcloud_L_device_;
};

}  // namespace nvblox

#include "nvblox_ros/impl/nvblox_node_impl.hpp"

#endif  // NVBLOX_ROS__NVBLOX_NODE_HPP_
