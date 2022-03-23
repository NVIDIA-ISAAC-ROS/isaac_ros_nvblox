/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <glog/logging.h>

#include <nvblox/datasets/image_loader.h>
#include <nvblox/datasets/parse_3dmatch.h>
#include <nvblox/nvblox.h>
#include <nvblox/utils/timing.h>


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

#include "nvblox_ros/conversions.hpp"

namespace nvblox
{

class Nvblox3DMatchNode : public rclcpp::Node
{
public:
  Nvblox3DMatchNode();

  /// Get the next frame in the dataset.
  void timerCallback();

  /// Integrate a particular frame number.
  bool integrateFrame(const int frame_number);

private:
  /// Sets up publishing and subscribing. Should only be called from
  /// constructor.
  void setupRos();

  /// Publish markers for visualization.
  rclcpp::Publisher<nvblox_msgs::msg::Mesh>::SharedPtr mesh_publisher_;

  /// Timers.
  rclcpp::TimerBase::SharedPtr update_timer_;

  /// Converer.
  RosConverter converter_;

  /// Frame IDs
  std::string map_frame_id_ = "map";
  std::string camera_frame_id_ = "camera";

  /// Dataset settings.
  std::string base_path_;
  float voxel_size_ = 0.2;
  int sequence_num_ = 1;
  int frame_number_ = 0;
  bool rotate_optical_frame_ = false;
  bool rotate_world_frame_ = true;

  /// NVblox layers.
  std::shared_ptr<nvblox::TsdfLayer> tsdf_layer_;
  std::shared_ptr<nvblox::EsdfLayer> esdf_layer_;
  std::shared_ptr<nvblox::MeshLayer> mesh_layer_;

  // Integrators.
  ProjectiveTsdfIntegrator tsdf_integrator_;
  MeshIntegrator mesh_integrator_;
};

Nvblox3DMatchNode::Nvblox3DMatchNode()
: Node("nvblox_3dmatch_node")
{
  setupRos();
}

void Nvblox3DMatchNode::setupRos()
{
  double time_between_frames = 0.5;  // seconds
  time_between_frames =
    declare_parameter<float>("time_between_frames", time_between_frames);

  base_path_ = declare_parameter<std::string>("path", base_path_);

  if (base_path_.empty()) {
    RCLCPP_ERROR(get_logger(), "No base path specified!");
    return;
  }

  // Mesh publishing
  mesh_publisher_ = this->create_publisher<nvblox_msgs::msg::Mesh>("mesh", 1);

  // Image settings
  rotate_optical_frame_ =
    declare_parameter<bool>("rotate_optical_frame", rotate_optical_frame_);
  rotate_world_frame_ =
    declare_parameter<bool>("rotate_world_frame", rotate_world_frame_);

  // Create the layers.
  voxel_size_ = declare_parameter<float>("voxel_size", voxel_size_);

  // Initialize the layers.
  const float block_size = voxel_size_ * VoxelBlock<bool>::kVoxelsPerSide;

  tsdf_layer_.reset(new TsdfLayer(voxel_size_, MemoryType::kDevice));
  esdf_layer_.reset(new EsdfLayer(voxel_size_, MemoryType::kUnified));
  mesh_layer_.reset(new MeshLayer(block_size, MemoryType::kUnified));

  mesh_integrator_.min_weight() = 2.0f;

  // Create a timer to load a new frame every n seconds.
  update_timer_ =
    create_wall_timer(
    std::chrono::duration<float>(time_between_frames),
    std::bind(&Nvblox3DMatchNode::timerCallback, this));
}

void Nvblox3DMatchNode::timerCallback()
{
  if (integrateFrame(frame_number_)) {
    RCLCPP_INFO(get_logger(), "Outputting frame numer %d", frame_number_);
    frame_number_++;
  } else {
    // Kill timer.
    RCLCPP_INFO(get_logger(), "Finished dataset.");
    update_timer_->cancel();
  }
}

bool Nvblox3DMatchNode::integrateFrame(const int frame_number)
{
  if (!tsdf_layer_) {
    RCLCPP_ERROR(
      get_logger(),
      "No layer created. Please create a layer first.");
    return false;
  }

  timing::Timer timer_file("file_loading");

  // Get the camera for this frame.
  Eigen::Matrix3f camera_intrinsics;
  if (!datasets::threedmatch::parseCameraFromFile(
      datasets::threedmatch::getPathForCameraIntrinsics(base_path_),
      &camera_intrinsics))
  {
    return false;
  }

  // Load the image into a Depth Frame.
  DepthImage depth_image;
  if (!datasets::load16BitDepthImage(
      datasets::threedmatch::getPathForDepthImage(
        base_path_, sequence_num_,
        frame_number),
      &depth_image))
  {
    return false;
  }
  // Get the transform.
  Transform T_L_C;
  if (!datasets::threedmatch::parsePoseFromFile(
      datasets::threedmatch::getPathForFramePose(
        base_path_, sequence_num_,
        frame_number),
      &T_L_C))
  {
    return false;
  }

  // Create a camera object.
  int image_width = depth_image.cols();
  int image_height = depth_image.rows();
  float fu = camera_intrinsics(0, 0);
  float fv = camera_intrinsics(1, 1);
  float cu = camera_intrinsics(0, 2);
  float cv = camera_intrinsics(1, 2);

  Camera camera(fu, fv, cu, cv, image_width, image_height);

  timer_file.Stop();

  // Rotate the camera frame to be an optical frame.
  if (rotate_optical_frame_) {
    RCLCPP_INFO(get_logger(), "Rotating optical.");
    Eigen::Matrix3f rotation;
    rotation << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    T_L_C = T_L_C.rotate(rotation);
  }

  // Rotate the world frame since Y is up in the normal 3D match dasets.
  if (rotate_world_frame_) {
    RCLCPP_INFO(get_logger(), "Rotating world.");
    Eigen::Quaternionf q_L_O = Eigen::Quaternionf::FromTwoVectors(
      Vector3f(0, 1, 0), Vector3f(0, 0, 1));
    T_L_C = q_L_O * T_L_C;
  }

  // Publish the TF pose.
  /*
  geometry_msgs::TransformStamped tf_stamped;
  tf_stamped = tf2::eigenToTransform(T_L_C.cast<double>());
  tf_stamped.header.stamp = ros::Time::now();
  tf_stamped.header.frame_id = map_frame_id_;
  tf_stamped.child_frame_id = camera_frame_id_;
  tf_broadcaster_.sendTransform(tf_stamped); */

  // Finally, processing.
  timing::Timer timer_integrate("integrate");

  // Call the integrator.
  std::vector<Index3D> updated_blocks;

  tsdf_integrator_.integrateFrame(
    depth_image, T_L_C, camera, tsdf_layer_.get(),
    &updated_blocks);
  // Mesh integrator
  mesh_integrator_.integrateBlocksGPU(
    *tsdf_layer_, updated_blocks,
    mesh_layer_.get());

  timer_integrate.Stop();

  // Publish the mesh updates.
  nvblox_msgs::msg::Mesh mesh_msg;
  converter_.meshMessageFromMeshBlocks(*mesh_layer_, updated_blocks, &mesh_msg);

  mesh_msg.header.frame_id = map_frame_id_;
  mesh_msg.header.stamp = get_clock()->now();
  mesh_publisher_->publish(mesh_msg);
  RCLCPP_INFO(get_logger(), "Published a message.");
  return true;
}

}  // namespace nvblox

int main(int argc, char ** argv)
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nvblox::Nvblox3DMatchNode>());
  rclcpp::shutdown();

  std::cout << "Timings: " << nvblox::timing::Timing::Print();

  return 0;
}
