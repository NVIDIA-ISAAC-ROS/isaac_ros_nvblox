/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef REALSENSE_SPLITTER__QOS_HPP_
#define REALSENSE_SPLITTER__QOS_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace nvblox
{

rmw_qos_profile_t parseQosString(const std::string & str);

}  // namespace nvblox

#endif  // REALSENSE_SPLITTER__QOS_HPP_
