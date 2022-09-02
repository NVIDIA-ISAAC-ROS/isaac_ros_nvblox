/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

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

#include "nvblox_ros/conversions.hpp"
#include "nvblox_ros/transformer.hpp"

namespace nvblox
{

class NvbloxNode : public rclcpp::Node
{
public:
  NvbloxNode();
  virtual ~NvbloxNode() = default;

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
  void processDepthQueue();
  void processColorQueue();
  void processPointcloudQueue();
  void processEsdf();
  virtual void processMesh();

  // Process a single images
  virtual bool processDepthImage(
    sensor_msgs::msg::Image::ConstSharedPtr & depth_img_ptr,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);
  virtual bool processColorImage(
    sensor_msgs::msg::Image::ConstSharedPtr & color_img_ptr,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);
  virtual bool processLidarPointcloud(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_ptr);

  bool canTransform(const std_msgs::msg::Header & header);

  void publishSlicePlane(const rclcpp::Time & timestamp, const Transform & T_S_C);

private:
  // Helper functions to make the code more readable.
  void updateEsdf(const rclcpp::Time & timestamp);
  void updateMesh(const rclcpp::Time & timestamp);

  // Map clearing
  void clearMapOutsideOfRadius(const std::string & target_frame_id, const rclcpp::Time & timestamp);


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
    pointcloud_publisher_;
  rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr
    map_slice_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr slice_bounds_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    mesh_marker_publisher_;

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
  rclcpp::TimerBase::SharedPtr esdf_processing_timer_;
  rclcpp::TimerBase::SharedPtr mesh_processing_timer_;

  // ROS & nvblox settings
  float voxel_size_ = 0.05f;
  bool esdf_ = true;
  bool esdf_2d_ = true;
  bool distance_slice_ = true;
  bool mesh_ = true;
  float slice_height_ = 1.0f;

  // Depth / Lidar / color toggle parameters
  bool use_depth_ = true;
  bool use_lidar_ = true;
  bool use_color_ = true;

  // LIDAR settings, defaults for Velodyne VLP16
  int lidar_width_ = 1800;
  int lidar_height_ = 16;
  float lidar_vertical_fov_rad_ = 30.0 * M_PI / 180.0;

  // Used for ESDF slicing. Everything between min and max height will be
  // compressed to a single 2D level, output at slice_height_.
  float min_height_ = 0.0f;
  float max_height_ = 1.0f;

  // Slice visualization params
  std::string slice_visualization_attachment_frame_id_ = "base_link";
  float slice_visualization_side_length_ = 10.0f;

  // ROS settings & update throttles
  std::string global_frame_ = "map";
  /// Pose frame to use if using transform topics.
  std::string pose_frame_ = "base_link";
  float max_tsdf_update_hz_ = 10.0f;
  float max_color_update_hz_ = 5.0f;
  float max_pointcloud_update_hz_ = 10.0f;
  float max_mesh_update_hz_ = 5.0f;
  float max_esdf_update_hz_ = 2.0f;
  /// Specifies what rate to poll the color & depth updates at.
  /// Will exit as no-op if no new images are in the queue so it is safe to
  /// set this higher than you expect images to come in at.
  float max_poll_rate_hz_ = 100.0f;

  /// Map clearing params
  /// Note that values <=0.0 indicate that no clearing is performed.
  float map_clearing_radius_m_ = -1.0f;

  // Mapper
  // Holds the map layers and their associated integrators
  // - TsdfLayer, ColorLayer, EsdfLayer, MeshLayer
  std::unique_ptr<RgbdMapper> mapper_;

  // The most important part: the ROS converter. Just holds buffers as state.
  RosConverter converter_;

  // Caches for GPU images
  ColorImage color_image_;
  DepthImage depth_image_;
  DepthImage pointcloud_image_;

  // Message statistics (useful for debugging)
  libstatistics_collector::topic_statistics_collector::
  ReceivedMessagePeriodCollector<sensor_msgs::msg::Image>
  depth_frame_statistics_;
  libstatistics_collector::topic_statistics_collector::
  ReceivedMessagePeriodCollector<sensor_msgs::msg::Image>
  rgb_frame_statistics_;

  // Output directory
  std::string output_dir_ = "";

  // State for integrators running at various speeds.
  rclcpp::Time last_tsdf_update_time_;
  rclcpp::Time last_color_update_time_;
  rclcpp::Time last_pointcloud_update_time_;
  rclcpp::Time last_esdf_update_time_;
  rclcpp::Time last_mesh_update_time_;

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

  // Keeps track of the mesh blocks deleted such that we can publish them for deletion in the rviz
  // plugin
  Index3DSet mesh_blocks_deleted_;
};

}  // namespace nvblox

#endif  // NVBLOX_ROS__NVBLOX_NODE_HPP_
