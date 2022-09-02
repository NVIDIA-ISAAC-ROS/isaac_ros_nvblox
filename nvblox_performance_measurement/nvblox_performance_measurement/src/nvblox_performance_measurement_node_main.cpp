/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "nvblox_performance_measurement/nvblox_performance_measurement_node.hpp"

using namespace nvblox;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NvbloxPerformanceMeasurementNode>());
  rclcpp::shutdown();
  return 0;
}
