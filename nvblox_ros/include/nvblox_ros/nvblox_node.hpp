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

#include <geometry_msgs/msg/vector3.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <chrono>
#include <list>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>
#include <unordered_map>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <nvblox_msgs/srv/file_path.hpp>
#include <nvblox_msgs/srv/esdf_and_gradients.hpp>

#include "nvblox_ros/layer_publishing.hpp"
#include "nvblox_ros/conversions/image_conversions.hpp"
#include "nvblox_ros/conversions/mesh_conversions.hpp"
#include "nvblox_ros/conversions/pointcloud_conversions.hpp"
#include "nvblox_ros/conversions/esdf_slice_conversions.hpp"
#include "nvblox_ros/conversions/esdf_and_gradients_conversions.hpp"
#include "nvblox_ros/mapper_initialization.hpp"
#include "nvblox_ros/transformer.hpp"
#include "nvblox_ros/camera_cache.hpp"
#include "nvblox_ros/nitros_types.hpp"
#include "nvblox_ros/node_params.hpp"
#include "nvblox_ros/service_request_task.hpp"

#include "isaac_ros_managed_nitros/managed_nitros_message_filters_subscriber.hpp"
#include "isaac_ros_managed_nitros/managed_nitros_publisher.hpp"
#include "isaac_ros_managed_nitros/managed_nitros_subscriber.hpp"
#include "isaac_ros_nitros_camera_info_type/nitros_camera_info.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"
#include "isaac_ros_nitros_image_type/nitros_image_view.hpp"

namespace nvblox
{

constexpr int8_t kOccupancyGridUnknownValue = -1;

class NvbloxNode : public rclcpp::Node
{
public:
  explicit NvbloxNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    const std::string & node_name = "nvblox_node", std::shared_ptr<CudaStream> cuda_stream =
    std::make_shared<CudaStreamOwning>());
  virtual ~NvbloxNode();

  // Setup. These are called by the constructor.
  void initializeMultiMapper();
  void subscribeToTopics();
  void advertiseTopics();
  void advertiseServices();
  void setupTimers();

  // Internal types for passing around images, their matching
  // segmentation masks, as well as the camera intrinsics.
  using ImageSegmentationMaskMsgTuple =
    std::tuple<NitrosViewPtr,
      sensor_msgs::msg::CameraInfo::ConstSharedPtr,
      NitrosViewPtr,
      sensor_msgs::msg::CameraInfo::ConstSharedPtr>;

  using ImageMsgTuple =
    std::tuple<NitrosViewPtr,
      sensor_msgs::msg::CameraInfo::ConstSharedPtr>;

  // Expresses the various types of Images that can be queued in the node for processing.
  using ImageTypeVariant = std::variant<ImageMsgTuple, ImageSegmentationMaskMsgTuple>;

  // Internal type of an image message with an *optional* mask.
  using ImageMsgOptionalMaskMsgTuple =
    std::tuple<NitrosViewPtr, sensor_msgs::msg::CameraInfo::ConstSharedPtr,
      std::optional<NitrosViewPtr>,
      std::optional<sensor_msgs::msg::CameraInfo::ConstSharedPtr>>;

  // Named indices for the MsgTuple members.
  static constexpr size_t kMsgTupleImageIdx = 0;
  static constexpr size_t kMsgTupleCameraInfoIdx = 1;
  static constexpr size_t kMsgTupleMaskIdx = 2;
  static constexpr size_t kMsgTupleMaskCameraInfoIdx = 3;

  // Callback functions. These just stick images in a queue.
  void depthPlusMaskImageCallback(
    const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & depth_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & depth_camera_info,
    const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & seg_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & seg_camera_info);
  void depthImageCallback(
    const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & depth_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & depth_camera_info);
  void colorPlusMaskImageCallback(
    const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & color_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & color_camera_info,
    const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & seg_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & seg_camera_info);
  void colorImageCallback(
    const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & color_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & color_camera_info);
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

  // Publish debug visualizations for rviz
  void publishDebugVisualizations();

  // Process data
  virtual bool processDepthImage(const ImageTypeVariant & depth_mask_msg);
  virtual bool processColorImage(const ImageTypeVariant & color_mask_msg);
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
  virtual void processServiceRequestTaskQueue();
  virtual void processEsdf();

  // Return true if the time between the two passed timestamps is sufficient to trigger an action
  // under the requested rate.
  bool shouldProcess(
    const rclcpp::Time & time_now, const rclcpp::Time & time_last,
    const float desired_frequency_hz);

  // Publish the dynamic outputs
  void publishDynamics(
    const std::string & camera_frame_id);

  // Publish human debug output
  void publishHumanDebugOutput(const std::string & camera_frame_id, const Camera & camera);

  // Publish the back projected depth image for debug purposes
  void publishBackProjectedDepth(
    const Camera & camera, const std::string & frame, const rclcpp::Time & timestamp);

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
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & occupancy_grid_publisher,
    const float unknown_value, const Mapper * mapper_2 = nullptr);

  // Publish an occupancy grid message from a slice image.
  void publishOccupancyGridMsg(
    const float voxel_size, const int width,
    const int height, const double origin_x_position,
    const double origin_y_position, const nvblox::Image<float> & map_slice_image,
    const float unknown_value,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & occupancy_grid_publisher);

  // Map clearing
  void clearMapOutsideOfRadiusOfLastKnownPose();

  /// Used by callbacks (internally) to add input items to queues.
  /// @tparam QueuedType The type of the input item stored by the queue.
  /// @param queue_name Name of the queue, used for logging.
  /// @param item Item to be added to the queue.
  /// @param queue_ptr Queue where to add the item.
  /// @param queue_mutex_ptr Mutex protecting the queue.
  template<typename QueuedType>
  void pushOntoQueue(
    const std::string & queue_name,
    QueuedType item,
    std::unique_ptr<std::list<QueuedType>> & queue_ptr,
    std::mutex * queue_mutex_ptr);

  // Used internally to unify processing of queues
  // that process an input item based on a ready check.
  template<typename QueuedType>
  using ProcessFunctionType = std::function<bool (const QueuedType &)>;
  template<typename QueuedType>
  using ReadyCheckFunctionType = std::function<bool (const QueuedType &)>;

  /// Returns true if pose(s) for the image message is available.
  /// @param variant_msg The image(s) + cam_info message.
  /// @return True if poses are available.
  bool isPoseAvailable(const ImageTypeVariant & variant_msg);

  /// Takes an ImageTypeVariant and decomposes it into an image with an optional mask.
  /// @param msg The image(s) + cam_info message.
  /// @return A tuple of an image, camera_info, and optional mask.
  ImageMsgOptionalMaskMsgTuple decomposeImageTypeVariant(const ImageTypeVariant & msg);

  /// Processes a queue of input items by detecting if they're ready and then
  /// calling a process function.
  /// @tparam QueuedType The type of the input items in the queue.
  /// @param queue_ptr Queue of input items to process.
  /// @param queue_mutex_ptr Mutex protecting the queue.
  /// @param ready_check_function Function called on each item to check if
  /// it's ready to be processed
  /// @param process_function Function to process each ready item.
  template<typename QueuedType>
  void processQueue(
    std::unique_ptr<std::list<QueuedType>> & queue_ptr, std::mutex * queue_mutex_ptr,
    ReadyCheckFunctionType<QueuedType> ready_check_function,
    ProcessFunctionType<QueuedType> process_function);

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

  /// Color topics to listen to.
  static constexpr std::array<const char *,
    kMaxNumCameras> kSegTopicBaseNames =
  {
    // Multi-camera topics
    "camera_0/mask",
    "camera_1/mask",
    "camera_2/mask",
    "camera_3/mask",
  };


  /// Image + info subscribers
  std::vector<std::shared_ptr<nvidia::isaac_ros::nitros::message_filters::Subscriber<NitrosView>>>
  depth_image_subs_;
  std::vector<std::shared_ptr<::message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>>
  depth_camera_info_subs_;
  std::vector<std::shared_ptr<nvidia::isaac_ros::nitros::message_filters::Subscriber<NitrosView>>>
  color_image_subs_;
  std::vector<std::shared_ptr<::message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>>
  color_camera_info_subs_;
  std::vector<std::shared_ptr<nvidia::isaac_ros::nitros::message_filters::Subscriber<NitrosView>>>
  segmentation_image_subs_;
  std::vector<std::shared_ptr<::message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>>
  segmentation_camera_info_subs_;

  // Sync Policies
  using image_mask_approx_policy = ::message_filters::sync_policies::ApproximateTime<
    nvidia::isaac_ros::nitros::NitrosImage, sensor_msgs::msg::CameraInfo,
    nvidia::isaac_ros::nitros::NitrosImage, sensor_msgs::msg::CameraInfo>;
  using image_mask_approx_sync = ::message_filters::Synchronizer<image_mask_approx_policy>;

  using image_exact_policy = ::message_filters::sync_policies::ExactTime<
    nvidia::isaac_ros::nitros::NitrosImage, sensor_msgs::msg::CameraInfo>;
  using image_exact_sync = ::message_filters::Synchronizer<image_exact_policy>;

  using image_mask_exact_policy = ::message_filters::sync_policies::ExactTime<
    nvidia::isaac_ros::nitros::NitrosImage, sensor_msgs::msg::CameraInfo,
    nvidia::isaac_ros::nitros::NitrosImage, sensor_msgs::msg::CameraInfo>;
  using image_mask_exact_sync = ::message_filters::Synchronizer<image_mask_exact_policy>;

  std::vector<std::shared_ptr<image_mask_approx_sync>> timesync_depth_mask_;
  std::vector<std::shared_ptr<image_mask_exact_sync>> timesync_color_mask_;
  std::vector<std::shared_ptr<image_exact_sync>> timesync_depth_;
  std::vector<std::shared_ptr<image_exact_sync>> timesync_color_;


  // Pointcloud sub.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    pointcloud_sub_;

  // Optional transform subs.
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
    transform_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // Publishers
  std::unique_ptr<LayerPublisher> layer_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    static_esdf_pointcloud_publisher_;
  rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr
    static_map_slice_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    pessimistic_static_esdf_pointcloud_publisher_;
  rclcpp::Publisher<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr
    pessimistic_static_map_slice_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    esdf_slice_bounds_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    workspace_bounds_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    shapes_to_clear_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    color_frame_overlay_publisher_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
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
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    static_occupancy_grid_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    dynamic_occupancy_grid_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    combined_occupancy_grid_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
    tsdf_zero_crossings_ground_plane_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    tsdf_zero_crossings_pointcloud_publisher_;

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

  // Collection of params for the nvblox node
  NvbloxNodeParams params_;

  // Counter for back projection subsampling
  std::map<std::string, uint32_t> back_projection_idx_;

  /// The last time processing occurred. Used to maintain rates.
  /// NOTE: Because we support multiple cameras, the first two maps map from the frame_id
  /// of the camera to the timestamp of the last image integrated from that camera.
  std::unordered_map<std::string, rclcpp::Time> integrate_depth_last_times_;
  std::unordered_map<std::string, rclcpp::Time> integrate_color_last_times_;
  rclcpp::Time integrate_lidar_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time update_mesh_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time update_esdf_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time publish_layer_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time publish_debug_vis_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time decay_tsdf_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time decay_dynamic_occupancy_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time clear_map_outside_radius_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  /// The time stamp of the last frame contributing to the reconstruction.
  rclcpp::Time newest_integrated_depth_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  /// The time stamp when we last cleared shapes in the reconstruction.
  rclcpp::Time shape_clearing_last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

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
  conversions::PointcloudConverter pointcloud_converter_;
  conversions::EsdfSliceConverter esdf_slice_converter_;
  conversions::EsdfAndGradientsConverter esdf_and_gradients_converter_;

  // Caches for GPU images
  ColorImage color_image_{MemoryType::kDevice};
  DepthImage depth_image_{MemoryType::kDevice};
  MonoImage mask_image_{MemoryType::kDevice};
  DepthImage pointcloud_image_{MemoryType::kDevice};
  Image<conversions::Rgb> rgb_image_tmp_{MemoryType::kDevice};

  // Object for back projecting image to a pointcloud.
  DepthImageBackProjector image_back_projector_;

  // Cache the last known number of subscribers.
  size_t mesh_subscriber_count_ = 0;

  // Whether a service call requested the layers to be visualized.
  bool publish_layers_requested_ = false;

  // Types of items queued in service call queues.
  using EsdfServiceQueuedType = std::shared_ptr<ServiceRequestTask<NvbloxNode,
      nvblox_msgs::srv::EsdfAndGradients>>;
  using FilePathServiceQueuedType = std::shared_ptr<ServiceRequestTask<NvbloxNode,
      nvblox_msgs::srv::FilePath>>;

  // Input queues. Unique pointers are used to enable more flexibility when deallocating.
  std::unique_ptr<std::list<sensor_msgs::msg::PointCloud2::ConstSharedPtr>> pointcloud_queue_;
  std::unique_ptr<std::list<EsdfServiceQueuedType>> esdf_service_queue_;
  std::unique_ptr<std::list<FilePathServiceQueuedType>> file_path_service_queue_;
  std::unique_ptr<std::list<ImageTypeVariant>> depth_image_queue_;
  std::unique_ptr<std::list<ImageTypeVariant>>
  color_image_queue_;

  // Input queue names.
  static constexpr char kDepthQueueName[] = "depth_queue";
  static constexpr char kColorQueueName[] = "color_queue";
  static constexpr char kPointcloudQueueName[] = "pointcloud_queue";
  static constexpr char kFilePathServiceQueueName[] = "file_path_service_queue";
  static constexpr char kEsdfServiceQueueName[] = "esdf_service_queue";

  // Input queue mutexes.
  std::mutex depth_queue_mutex_;
  std::mutex color_queue_mutex_;
  std::mutex depth_mask_queue_mutex_;
  std::mutex color_mask_queue_mutex_;
  std::mutex pointcloud_queue_mutex_;
  std::mutex esdf_service_queue_mutex_;
  std::mutex file_path_service_queue_mutex_;

  // Counts the number of messages dropped from the input queues.
  // Maps the input queue to the number of messages that have been dropped.
  std::unordered_map<std::string, int> number_of_dropped_queued_items_;

  // Device caches
  Pointcloud pointcloud_C_device_;
  Pointcloud human_pointcloud_C_device_;
  Pointcloud human_pointcloud_L_device_;
  Pointcloud tsdf_zero_crossings_device_;

  Transform T_L_C_depth_;

  // The idle timer measures time spent *outside* the main tick function and can thus be used to
  // monitor how much headroom there is until the system gets saturated. A lower number <=1ms/tick
  // means that the system is under high pressure.
  static constexpr bool kStartStopped = true;
  timing::Timer idle_timer_{"ros/idle", kStartStopped};

  // Cuda stream for GPU work
  std::shared_ptr<CudaStream> cuda_stream_ = nullptr;

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
