/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "nvblox_ros/nvblox_node.hpp"

#include <nvblox/core/cuda/warmup.h>
#include <nvblox/io/mesh_io.h>
#include <nvblox/io/pointcloud_io.h>
#include <nvblox/utils/timing.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nvblox_ros/conversions.hpp"

using namespace std::chrono_literals;

namespace nvblox
{

NvbloxNode::NvbloxNode()
: Node("nvblox_node"), transformer_(this)
{
  // Declare & initialize the parameters.
  voxel_size_ = declare_parameter<float>("voxel_size", voxel_size_);
  global_frame_ = declare_parameter<std::string>("global_frame", global_frame_);
  pose_frame_ = declare_parameter<std::string>("pose_frame", pose_frame_);
  mesh_ = declare_parameter<bool>("mesh", mesh_);
  esdf_ = declare_parameter<bool>("esdf", esdf_);
  esdf_2d_ = declare_parameter<bool>("esdf_2d", esdf_2d_);
  distance_slice_ = declare_parameter<bool>("distance_slice", distance_slice_);
  slice_height_ = declare_parameter<float>("slice_height", slice_height_);
  min_height_ = declare_parameter<float>("min_height", min_height_);
  max_height_ = declare_parameter<float>("max_height", max_height_);
  // Update rates
  max_tsdf_update_hz_ =
    declare_parameter<float>("max_tsdf_update_hz", max_tsdf_update_hz_);
  max_color_update_hz_ =
    declare_parameter<float>("max_color_update_hz", max_color_update_hz_);
  max_mesh_update_hz_ =
    declare_parameter<float>("max_mesh_update_hz", max_mesh_update_hz_);
  max_esdf_update_hz_ =
    declare_parameter<float>("max_esdf_update_hz", max_esdf_update_hz_);

  // Set the transformer settings.
  transformer_.set_global_frame(global_frame_);
  transformer_.set_pose_frame(pose_frame_);

  // Initialize the map
  mapper_ = std::make_unique<RgbdMapper>(voxel_size_);

  // Subscribe to synchronized depth + cam_info topics
  depth_sub_.subscribe(this, "depth/image");
  depth_camera_info_sub_.subscribe(this, "depth/camera_info");

  constexpr int kQueueSize = 2;
  timesync_depth_.reset(
    new message_filters::Synchronizer<time_policy_t>(
      time_policy_t(kQueueSize), depth_sub_, depth_camera_info_sub_));
  timesync_depth_->registerCallback(
    std::bind(
      &NvbloxNode::depthImageCallback,
      this, std::placeholders::_1,
      std::placeholders::_2));

  // Subscribe to synchronized color + cam_info topics
  color_sub_.subscribe(this, "color/image");
  color_camera_info_sub_.subscribe(this, "color/camera_info");

  timesync_color_.reset(
    new message_filters::Synchronizer<time_policy_t>(
      time_policy_t(kQueueSize), color_sub_, color_camera_info_sub_));
  timesync_color_->registerCallback(
    std::bind(
      &NvbloxNode::colorImageCallback,
      this, std::placeholders::_1,
      std::placeholders::_2));

  // Subscribe to transforms.
  transform_sub_ =
    this->create_subscription<geometry_msgs::msg::TransformStamped>(
    "transform", 10,
    std::bind(
      &Transformer::transformCallback, &transformer_,
      std::placeholders::_1));
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "pose", 10,
    std::bind(
      &Transformer::poseCallback, &transformer_,
      std::placeholders::_1));

  // Publishers
  mesh_publisher_ = create_publisher<nvblox_msgs::msg::Mesh>("~/mesh", 1);
  pointcloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/pointcloud", 1);
  map_slice_publisher_ =
    create_publisher<nvblox_msgs::msg::DistanceMapSlice>("~/map_slice", 1);

  // Services
  save_ply_service_ = create_service<std_srvs::srv::Empty>(
    "~/save_ply", std::bind(
      &NvbloxNode::savePly, this, std::placeholders::_1,
      std::placeholders::_2));

  // Integrator settings.
  mapper_->tsdf_integrator().max_integration_distance_m(
    declare_parameter<float>(
      "tsdf_integrator_max_integration_distance_m",
      mapper_->tsdf_integrator().max_integration_distance_m()));
  mapper_->tsdf_integrator().truncation_distance_vox(
    declare_parameter<float>(
      "tsdf_integrator_truncation_distance_vox",
      mapper_->tsdf_integrator().truncation_distance_vox()));
  mapper_->tsdf_integrator().max_weight(
    declare_parameter<float>(
      "tsdf_integrator_max_weight", mapper_->tsdf_integrator().max_weight()));
  mapper_->mesh_integrator().min_weight() = declare_parameter<float>(
    "mesh_integrator_min_weight", mapper_->mesh_integrator().min_weight());
  mapper_->mesh_integrator().weld_vertices() =
    declare_parameter<bool>(
    "mesh_integrator_weld_vertices",
    mapper_->mesh_integrator().weld_vertices());
  mapper_->color_integrator().max_integration_distance_m(
    declare_parameter<float>(
      "color_integrator_max_integration_distance_m",
      mapper_->color_integrator().max_integration_distance_m()));
  mapper_->esdf_integrator().min_weight() = declare_parameter<float>(
    "esdf_integrator_min_weight", mapper_->esdf_integrator().min_weight());
  mapper_->esdf_integrator().min_site_distance_vox() = declare_parameter<float>(
    "esdf_integrator_min_site_distance_vox",
    mapper_->esdf_integrator().min_site_distance_vox());
  mapper_->esdf_integrator().min_site_distance_vox() =
    mapper_->esdf_integrator().max_distance_m() =
    declare_parameter<float>(
    "esdf_integrator_max_distance_m",
    mapper_->esdf_integrator().max_distance_m());

  // Where to put saved stuff
  output_dir_ = declare_parameter<std::string>("output_dir", output_dir_);

  RCLCPP_INFO(
    get_logger(),
    "Outputting results (as requested) to: " + output_dir_);

  // Start the message statistics
  depth_frame_statistics_.Start();
  rgb_frame_statistics_.Start();

  RCLCPP_INFO_STREAM(
    get_logger(), "Started up nvblox node in frame " <<
      global_frame_ << " and voxel size " <<
      voxel_size_);

  // Set state.
  last_tsdf_update_time_ = rclcpp::Time(0ul, get_clock()->get_clock_type());
  last_color_update_time_ = rclcpp::Time(0ul, get_clock()->get_clock_type());
  last_esdf_update_time_ = rclcpp::Time(0ul, get_clock()->get_clock_type());
  last_mesh_update_time_ = rclcpp::Time(0ul, get_clock()->get_clock_type());
}

void NvbloxNode::depthImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_img_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  // Message statistics
  depth_frame_statistics_.OnMessageReceived(
    *depth_img_ptr, this->get_clock()->now().nanoseconds());
  constexpr int kPublishPeriodMs = 10000;
  auto & clk = *this->get_clock();
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), clk, kPublishPeriodMs,
    "Depth frame statistics: \n" +
    libstatistics_collector::moving_average_statistics::
    StatisticsDataToString(
      depth_frame_statistics_.GetStatisticsResults()));

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), clk, kPublishPeriodMs,
    "Timing statistics: \n" + nvblox::timing::Timing::Print());

  timing::Timer ros_total_timer("ros/total");

  // Cache clock_now.
  rclcpp::Time clock_now = depth_img_ptr->header.stamp;

  if (max_tsdf_update_hz_ > 0.0f &&
    (clock_now - last_tsdf_update_time_).seconds() <
    1.0f / max_tsdf_update_hz_)
  {
    // Skip integrating this.
    return;
  }
  last_tsdf_update_time_ = clock_now;

  // Push it into the queue.
  depth_image_queue_.emplace_back(depth_img_ptr, camera_info_msg);
  processDepthQueue();
}

void NvbloxNode::colorImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & color_image_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  // Message statistics
  rgb_frame_statistics_.OnMessageReceived(
    *color_image_ptr, this->get_clock()->now().nanoseconds());
  constexpr int kPublishPeriodMs = 10000;
  auto & clk = *this->get_clock();
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), clk, kPublishPeriodMs,
    "RGB frame statistics: \n" +
    libstatistics_collector::moving_average_statistics::
    StatisticsDataToString(
      rgb_frame_statistics_.GetStatisticsResults()));

  timing::Timer ros_total_timer("ros/total");

  // Cache clock_now.
  rclcpp::Time clock_now = color_image_ptr->header.stamp;

  if (max_color_update_hz_ > 0.0f &&
    (clock_now - last_color_update_time_).seconds() <
    1.0f / max_color_update_hz_)
  {
    // Skip integrating this.
    return;
  }
  last_color_update_time_ = clock_now;

  // Push it into the queue.
  color_image_queue_.emplace_back(color_image_ptr, camera_info_msg);
  processColorQueue();
}

void NvbloxNode::processDepthQueue()
{
  timing::Timer ros_tsdf_timer("ros/tsdf");

  auto it = depth_image_queue_.begin();
  auto it_first_valid = depth_image_queue_.end();
  auto it_last_valid = depth_image_queue_.begin();

  while (++it != depth_image_queue_.end()) {
    sensor_msgs::msg::Image::ConstSharedPtr depth_img_ptr = it->first;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg = it->second;

    rclcpp::Time clock_now = depth_img_ptr->header.stamp;

    timing::Timer transform_timer("ros/tsdf/transform");
    // Get the TF for this image.
    Transform T_S_C;
    std::string target_frame = depth_img_ptr->header.frame_id;

    if (!transformer_.lookupTransformToGlobalFrame(
        target_frame, depth_img_ptr->header.stamp, &T_S_C))
    {
      continue;
    }
    transform_timer.Stop();

    timing::Timer conversions_timer("ros/tsdf/conversions");
    // Convert camera info message to camera object.
    Camera camera = converter_.cameraFromMessage(*camera_info_msg);

    // Convert the depth image.
    if (!converter_.depthImageFromImageMessage(depth_img_ptr, &depth_image_)) {
      RCLCPP_ERROR(get_logger(), "Failed to transform depth image.");
      continue;
    }
    conversions_timer.Stop();

    // Integrate
    timing::Timer integration_timer("ros/tsdf/integrate");
    mapper_->integrateDepth(depth_image_, T_S_C, camera);
    integration_timer.Stop();
    ros_tsdf_timer.Stop();

    // Esdf integrator (if enabled)
    if (esdf_) {
      // Check if it's been long enough since the last frame.
      if (max_esdf_update_hz_ <= 0.0f ||
        (clock_now - last_esdf_update_time_).seconds() >=
        1.0f / max_esdf_update_hz_)
      {
        last_esdf_update_time_ = clock_now;

        // Then do the update.
        // Otherwise do nothing.
        updateEsdf(depth_img_ptr->header.stamp);
      }
    }

    // Mesh integrator
    if (mesh_) {
      if (max_mesh_update_hz_ <= 0.0f ||
        (clock_now - last_mesh_update_time_).seconds() >=
        1.0f / max_mesh_update_hz_)
      {
        last_mesh_update_time_ = clock_now;
        updateMesh(depth_img_ptr->header.stamp);
      }
    }

    if (it_first_valid == depth_image_queue_.end()) {
      it_first_valid = it;
    }
    if (it_last_valid <= it) {
      it_last_valid = it;
    }
  }

  // Now we have 2 iterators pointing to what we want to delete.
  if (it_first_valid != depth_image_queue_.end()) {
    // Actually erase from the beginning of the queue.
    depth_image_queue_.erase(depth_image_queue_.begin(), ++it_last_valid);
  }
}

void NvbloxNode::processColorQueue()
{
  timing::Timer ros_color_timer("ros/color");

  auto it = color_image_queue_.begin();
  auto it_first_valid = color_image_queue_.end();
  auto it_last_valid = color_image_queue_.begin();

  while (++it != color_image_queue_.end()) {
    sensor_msgs::msg::Image::ConstSharedPtr color_image_ptr = it->first;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg = it->second;

    timing::Timer transform_timer("ros/color/transform");

    // Get the TF for this image.
    std::string target_frame = color_image_ptr->header.frame_id;
    Transform T_S_C;

    if (!transformer_.lookupTransformToGlobalFrame(
        target_frame, color_image_ptr->header.stamp, &T_S_C))
    {
      continue;
    }

    transform_timer.Stop();

    timing::Timer color_convert_timer("ros/color/conversion");

    // Convert camera info message to camera object.
    Camera camera = converter_.cameraFromMessage(*camera_info_msg);

    // Convert the color image.
    if (!converter_.colorImageFromImageMessage(
        color_image_ptr,
        &color_image_))
    {
      RCLCPP_ERROR(get_logger(), "Failed to transform color image.");
      continue;
    }
    color_convert_timer.Stop();

    // Integrate.
    timing::Timer color_integrate_timer("ros/color/integrate");
    mapper_->integrateColor(color_image_, T_S_C, camera);
    color_integrate_timer.Stop();

    if (it_first_valid == color_image_queue_.end()) {
      it_first_valid = it;
    }
    if (it_last_valid <= it) {
      it_last_valid = it;
    }
  }

  // Now we have 2 iterators pointing to what we want to delete.
  if (it_first_valid != color_image_queue_.end()) {
    // Actually erase from the beginning of the queue.
    color_image_queue_.erase(color_image_queue_.begin(), ++it_last_valid);
  }
}

void NvbloxNode::updateEsdf(const rclcpp::Time & timestamp)
{
  timing::Timer ros_esdf_timer("ros/esdf");

  timing::Timer esdf_integration_timer("ros/esdf/integrate");

  if (esdf_2d_) {
    mapper_->updateEsdfSlice(min_height_, max_height_, slice_height_);
  } else {
    mapper_->updateEsdf();
  }

  esdf_integration_timer.Stop();

  timing::Timer esdf_output_timer("ros/esdf/output");

  if (pointcloud_publisher_->get_subscription_count() > 0) {
    timing::Timer output_pointcloud_timer("ros/esdf/output/pointcloud");

    // Output the ESDF. Let's just do the full thing for now.
    sensor_msgs::msg::PointCloud2 pointcloud_msg;

    // AABB of a certain slice height.
    AxisAlignedBoundingBox aabb(Vector3f(
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        slice_height_ - voxel_size_ / 2.0f),
      Vector3f(
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        slice_height_ + voxel_size_ / 2.0f));

    converter_.pointcloudFromLayerInAABB(
      mapper_->esdf_layer(), aabb,
      &pointcloud_msg);

    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = timestamp;
    pointcloud_publisher_->publish(pointcloud_msg);

    output_pointcloud_timer.Stop();
  }

  // Also publish the map slice.
  if (distance_slice_ && map_slice_publisher_->get_subscription_count() > 0) {
    timing::Timer output_map_slice_timer("ros/esdf/output/map_slice");

    nvblox_msgs::msg::DistanceMapSlice map_slice;

    converter_.distanceMapSliceFromLayer(
      mapper_->esdf_layer(), slice_height_,
      &map_slice);
    map_slice.header.frame_id = global_frame_;
    map_slice.header.stamp = timestamp;
    map_slice_publisher_->publish(map_slice);
  }
}

void NvbloxNode::updateMesh(const rclcpp::Time & timestamp)
{
  timing::Timer ros_mesh_timer("ros/mesh");

  timing::Timer mesh_integration_timer("ros/mesh/integrate_and_color");
  const std::vector<Index3D> mesh_updated_list = mapper_->updateMesh();
  mesh_integration_timer.Stop();

  // Publish the mesh updates.
  timing::Timer mesh_output_timer("ros/mesh/output");
  size_t new_subscriber_count = mesh_publisher_->get_subscription_count();
  if (new_subscriber_count > 0) {
    nvblox_msgs::msg::Mesh mesh_msg;
    // In case we have new subscribers, publish the ENTIRE map once.
    if (new_subscriber_count > mesh_subscriber_count_) {
      RCLCPP_INFO(get_logger(), "Got a new subscriber, sending entire map.");

      converter_.meshMessageFromMeshBlocks(
        mapper_->mesh_layer(), mapper_->mesh_layer().getAllBlockIndices(),
        &mesh_msg);
      mesh_msg.clear = true;
    } else {
      converter_.meshMessageFromMeshBlocks(
        mapper_->mesh_layer(),
        mesh_updated_list, &mesh_msg);
    }
    mesh_msg.header.frame_id = global_frame_;
    mesh_msg.header.stamp = timestamp;
    mesh_publisher_->publish(mesh_msg);
  }
  mesh_subscriber_count_ = new_subscriber_count;

  mesh_output_timer.Stop();
}

void NvbloxNode::savePly(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  io::outputVoxelLayerToPly(
    mapper_->tsdf_layer(),
    output_dir_ + "/ros2_tsdf.ply");
  io::outputVoxelLayerToPly(
    mapper_->esdf_layer(),
    output_dir_ + "/ros2_esdf.ply");
  io::outputMeshLayerToPly(
    mapper_->mesh_layer(),
    output_dir_ + "/ros2_mesh.ply");
  RCLCPP_INFO(get_logger(), "Output PLY files to " + output_dir_);
}

}  // namespace nvblox

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();
  rclcpp::init(argc, argv);

  // Warmup CUDA so it doesn't affect our timings *as* much for the first
  // CUDA call.
  nvblox::warmupCuda();

  std::shared_ptr<nvblox::NvbloxNode> node(new nvblox::NvbloxNode());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
