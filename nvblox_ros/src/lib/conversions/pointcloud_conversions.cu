// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>
#include <thrust/extrema.h>
#include <thrust/transform.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "nvblox_ros/conversions/pointcloud_conversions.hpp"

namespace nvblox {
namespace conversions {

void copyDevicePointcloudToMsg(
    const device_vector<PclPointXYZI>& pcl_pointcloud_device,
    sensor_msgs::msg::PointCloud2* pointcloud_msg) {
  // Copy into the pointcloud message.
  const int num_points = pcl_pointcloud_device.size();
  size_t output_num_bytes = sizeof(PclPointXYZI) * num_points;
  pointcloud_msg->data.resize(output_num_bytes);
  // Copy over all the points.
  cudaMemcpy(pointcloud_msg->data.data(), pcl_pointcloud_device.data(),
             output_num_bytes, cudaMemcpyDeviceToHost);

  // Fill the other fields in the pointcloud_msg message.
  pointcloud_msg->height = 1;
  pointcloud_msg->width = num_points;
  pointcloud_msg->point_step = sizeof(PclPointXYZI);
  pointcloud_msg->row_step = output_num_bytes;

  // Populate the fields.
  sensor_msgs::msg::PointField point_field;
  point_field.name = "x";
  point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_field.offset = 0;
  point_field.count = 1;

  pointcloud_msg->fields.push_back(point_field);
  point_field.name = "y";
  point_field.offset += sizeof(float);
  pointcloud_msg->fields.push_back(point_field);
  point_field.name = "z";
  point_field.offset += sizeof(float);
  pointcloud_msg->fields.push_back(point_field);
  point_field.name = "intensity";
  point_field.offset += sizeof(float);
  pointcloud_msg->fields.push_back(point_field);
}

PointcloudConverter::PointcloudConverter()
    : PointcloudConverter(std::make_shared<CudaStreamOwning>()) {}

PointcloudConverter::PointcloudConverter(
    std::shared_ptr<CudaStream> cuda_stream)
    : cuda_stream_(cuda_stream) {}

bool PointcloudConverter::checkLidarPointcloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud,
    const Lidar& lidar) {
  // Check the cache
  if (checked_lidar_models_.find(lidar) != checked_lidar_models_.end()) {
    return true;
  }

  // Go through the pointcloud and check that each point projects to a pixel
  // center.
  sensor_msgs::PointCloud2ConstIterator<float> iter_xyz(*pointcloud, "x");
  for (; iter_xyz != iter_xyz.end(); ++iter_xyz) {
    Vector3f point(iter_xyz[0], iter_xyz[1], iter_xyz[2]);
    if (point.hasNaN() || !lidar.isInValidRange(point)) {
      continue;
    }
    Vector2f u_C;
    if (!lidar.project(point, &u_C)) {
      // Point fell outside the FoV specified in the intrinsics.
      return false;
    }
  }
  checked_lidar_models_.insert(lidar);
  return true;
}

void PointcloudConverter::writeLidarPointcloudToFile(
    const std::string filepath_prefix,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud) {
  // Write the dimensions
  std::ofstream width_file(filepath_prefix + "_dims.txt", std::ofstream::out);
  width_file << pointcloud->width << ", " << pointcloud->height;
  width_file.close();
  // Write the data
  Eigen::MatrixX3f pointcloud_matrix(pointcloud->width * pointcloud->height, 3);
  sensor_msgs::PointCloud2ConstIterator<float> iter_xyz(*pointcloud, "x");
  int idx = 0;
  for (; iter_xyz != iter_xyz.end(); ++iter_xyz) {
    Vector3f point(iter_xyz[0], iter_xyz[1], iter_xyz[2]);
    pointcloud_matrix.row(idx) = point;
    idx++;
  }
  io::writeToCsv(filepath_prefix + ".csv", pointcloud_matrix);
}

__global__ void depthImageFromPointcloudKernel(
    const Vector3f* pointcloud,          // NOLINT
    const Lidar lidar,                   // NOLINT
    const int num_bytes_between_points,  // NOLINT
    const int num_points,                // NOLINT
    float* depth_image) {
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;

  if (idx >= num_points) {
    return;
  }

  // Read a point from global memory
  const Vector3f point = pointcloud[idx];

  if (isnan(point.x()) || isnan(point.y()) || isnan(point.z())) {
    return;
  }

  // Project
  Index2D u_C_int;
  if (!lidar.project(point, &u_C_int)) {
    return;
  }

  // Write the depth to the image
  // NOTE(alexmillane): It's possible multiple points project to the same pixel.
  // Firstly, writes should be atomic, so we're fine in that respect. The last
  // projection to write "wins" so we basically end up with a random value, of
  // the values that project to this pixel.
  image::access(u_C_int.y(), u_C_int.x(), lidar.num_azimuth_divisions(),
                depth_image) = lidar.getDepth(point);
}

struct Vector3fToPcl {
  __host__ __device__ PclPointXYZI operator()(const Vector3f& vec) const {
    PclPointXYZI point;
    point.x = vec.x();
    point.y = vec.y();
    point.z = vec.z();
    point.intensity = 1.0;
    return point;
  };
};

// Internal pointcloud representation to a ROS pointcloud
void PointcloudConverter::pointcloudMsgFromPointcloud(
    const Pointcloud& pointcloud,
    sensor_msgs::msg::PointCloud2* pointcloud_msg) {
  CHECK_NOTNULL(pointcloud_msg);
  CHECK(pointcloud.memory_type() == MemoryType::kDevice ||
        pointcloud.memory_type() == MemoryType::kUnified);

  pcl_pointcloud_device_.resizeAsync(pointcloud.size(), *cuda_stream_);

  thrust::transform(thrust::device, pointcloud.points().begin(),
                    pointcloud.points().end(), pcl_pointcloud_device_.begin(),
                    Vector3fToPcl());

  // Put the points in the message
  copyDevicePointcloudToMsg(pcl_pointcloud_device_, pointcloud_msg);
}

// Helper function to efficiently resize buffers with capacity expansion
template <typename T>
void resizeBufferWithCapacityExpansion(unified_vector<T>* buffer, int required_size,
                                const CudaStream& cuda_stream) {
  // Expand capacity if required (with expansion factor to reduce frequent
  // reallocations)
  if (required_size > buffer->capacity()) {
    constexpr float kBufferExpansionFactor = 1.5f;
    const int new_capacity =
        static_cast<int>(kBufferExpansionFactor * required_size);
    buffer->reserveAsync(new_capacity, cuda_stream);
  }
  // Always resize to match the exact required size
  buffer->resizeAsync(required_size, cuda_stream);
  cuda_stream.synchronize();
}

// ROS pointcloud to internal pointcloud representation
void PointcloudConverter::pointcloudFromPointcloudMsg(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
    Pointcloud* pointcloud, bool load_per_point_timestamps, bool pointcloud2_timestamps_are_relative) {
  CHECK_NOTNULL(pointcloud);

  // Copy the points from the message to the pointcloud
  copyPointsFromPointcloudMsgAsync(pointcloud_msg, pointcloud);

  // Optionally copy the timestamps from the message to the pointcloud
  if (load_per_point_timestamps) {
    copyTimestampsFromPointcloudMsgAsync(pointcloud_msg, pointcloud, pointcloud2_timestamps_are_relative);
  }

  cuda_stream_->synchronize();
}

void PointcloudConverter::pointsToCubesMarkerMsg(
    const std::vector<Vector3f>& points, const float cube_size,
    const Color& color, visualization_msgs::msg::Marker* marker_ptr) {
  CHECK_NOTNULL(marker_ptr);
  // Publish
  marker_ptr->action = marker_ptr->ADD;
  marker_ptr->type = marker_ptr->CUBE_LIST;
  marker_ptr->points.reserve(points.size());
  for (const Vector3f& p_L : points) {
    geometry_msgs::msg::Point point;
    point.x = p_L.x();
    point.y = p_L.y();
    point.z = p_L.z();
    marker_ptr->points.push_back(point);
  }
  marker_ptr->color.r = static_cast<float>(color.r()) / 255.0f;
  marker_ptr->color.g = static_cast<float>(color.g()) / 255.0f;
  marker_ptr->color.b = static_cast<float>(color.b()) / 255.0f;
  marker_ptr->scale.x = cube_size;
  marker_ptr->scale.y = cube_size;
  marker_ptr->scale.z = cube_size;
}

void PointcloudConverter::copyPointsFromPointcloudMsgAsync(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
  Pointcloud* pointcloud) {
    CHECK_NOTNULL(pointcloud);

    const int num_points = pointcloud_msg->width * pointcloud_msg->height;

    // Resize pointcloud host buffer
    resizeBufferWithCapacityExpansion(&points_host_, num_points, *cuda_stream_);
  
    // Copy points from ROS message to host buffer
    sensor_msgs::PointCloud2ConstIterator<float> iter_xyz(*pointcloud_msg, "x");
    int idx = 0;
    for (; iter_xyz != iter_xyz.end(); ++iter_xyz) {
      points_host_[idx][0] = iter_xyz[0];
      points_host_[idx][1] = iter_xyz[1];
      points_host_[idx][2] = iter_xyz[2];
      idx++;
    }

    // Copy the points to the pointcloud
    pointcloud->copyPointsFromAsync(points_host_, *cuda_stream_);
}


void PointcloudConverter::copyTimestampsFromPointcloudMsgAsync(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
  Pointcloud* pointcloud, bool pointcloud2_timestamps_are_relative) {
  CHECK_NOTNULL(pointcloud);

  // Check if timestamps are available in the pointcloud message
  // Common timestamp field names: "t", "timestamp", "time"
  bool has_timestamps = false;
  std::string timestamp_field;
  uint8_t timestamp_datatype;
  for (const auto& field : pointcloud_msg->fields) {
    if (field.name == "t" || field.name == "timestamp" || field.name == "time") {
      has_timestamps = true;
      timestamp_field = field.name;
      timestamp_datatype = field.datatype;
      break;
    }
  }
  CHECK(has_timestamps) << "No timestamps found in the pointcloud message";

  // Copy timestamps from the message to the pointcloud
  const int num_points = pointcloud_msg->width * pointcloud_msg->height;
  
  // Resize timestamps host buffer
  resizeBufferWithCapacityExpansion(&timestamps_host_, num_points, *cuda_stream_);

  // Lambda template to process timestamps based on type
  auto writeTimestampsToHostBuffer = [&]<typename TimestampType>() {
    // Get the iterator for the timestamps with the appropriate datatype
    sensor_msgs::PointCloud2ConstIterator<TimestampType> iter_t(*pointcloud_msg, timestamp_field);
    
    constexpr double kNanoSecondsToMilliSeconds = 1e-6;
    const rclcpp::Time header_timestamp = pointcloud_msg->header.stamp;

    // If the per-point timestamps are absolute, we need to subtract the header timestamp to make them relative.
    // Internally we expect the timestamps to be relative to the scan start.
    const int64_t offset_to_relative_ns = pointcloud2_timestamps_are_relative ? 0 : header_timestamp.nanoseconds();
    int idx = 0;
    for (; iter_t != iter_t.end(); ++iter_t) {
      const int64_t relative_timestamp_ns = static_cast<int64_t>(*iter_t) - offset_to_relative_ns;
      timestamps_host_[idx] = Time(static_cast<int64_t>(relative_timestamp_ns * kNanoSecondsToMilliSeconds));
      CHECK_GE(timestamps_host_[idx], Time(0)) 
          << "Relative timestamp must be positive. "
          << "Check whether the pointcloud2_timestamps_are_relative param should be set to true.";
      idx++;
    }
  };

  // Dispatch based on datatype
  switch (timestamp_datatype) {
    case sensor_msgs::msg::PointField::INT8:
      writeTimestampsToHostBuffer.operator()<int8_t>();
      break;
    case sensor_msgs::msg::PointField::UINT8:
      writeTimestampsToHostBuffer.operator()<uint8_t>();
      break;
    case sensor_msgs::msg::PointField::INT16:
      writeTimestampsToHostBuffer.operator()<int16_t>();
      break;
    case sensor_msgs::msg::PointField::UINT16:
      writeTimestampsToHostBuffer.operator()<uint16_t>();
      break;
    case sensor_msgs::msg::PointField::INT32:
      writeTimestampsToHostBuffer.operator()<int32_t>();
      break;
    case sensor_msgs::msg::PointField::UINT32:
      writeTimestampsToHostBuffer.operator()<uint32_t>();
      break;
    case sensor_msgs::msg::PointField::FLOAT32:
      writeTimestampsToHostBuffer.operator()<float>();
      break;
    case sensor_msgs::msg::PointField::FLOAT64:
      writeTimestampsToHostBuffer.operator()<double>();
      break;
    default:
      CHECK(false) << "Unsupported timestamp datatype: " << static_cast<int>(timestamp_datatype);
  }

  // Copy the timestamps to the pointcloud
  pointcloud->copyTimestampsFromAsync(timestamps_host_, *cuda_stream_);
}

Time getPointcloudScanDurationMs(const Pointcloud& pointcloud, const CudaStream& cuda_stream) {
  // Check if the pointcloud has timestamps
  if (!pointcloud.timestamps_ms().has_value() || 
      pointcloud.timestamps_ms().value().empty()) {
    return Time(0);
  }

  // Get pointer to timestamps on device
  const unified_vector<Time>& timestamps_device = pointcloud.timestamps_ms().value();
  const Time* timestamps_ptr = timestamps_device.data();
  const size_t num_timestamps = timestamps_device.size();

  // Cast to int64_t* since Time is just a wrapper around int64_t
  // and Thrust can handle primitive types
  const int64_t* timestamps_int64_ptr = reinterpret_cast<const int64_t*>(timestamps_ptr);

  // Use thrust::max_element on the int64_t values
  thrust::device_ptr<const int64_t> timestamps_begin(timestamps_int64_ptr);
  thrust::device_ptr<const int64_t> timestamps_end(timestamps_int64_ptr + num_timestamps);
  thrust::device_ptr<const int64_t> max_it = thrust::max_element(thrust::device.on(cuda_stream),
      timestamps_begin, timestamps_end);

  // Copy the max timestamp to host.
  const int64_t* max_ptr = thrust::raw_pointer_cast(max_it.get());
  int64_t max_timestamp_int64;
  cudaMemcpyAsync(&max_timestamp_int64, max_ptr,
                  sizeof(int64_t),
                  cudaMemcpyDefault,
                  cuda_stream);
  cuda_stream.synchronize();

  // The max relative scan timestamp is the scan duration.
  return Time(max_timestamp_int64);
}

}  // namespace conversions
}  // namespace nvblox
