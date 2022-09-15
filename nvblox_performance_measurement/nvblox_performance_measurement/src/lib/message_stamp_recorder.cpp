/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "nvblox_performance_measurement/message_stamp_recorder.hpp"

namespace nvblox
{

const std::vector<rclcpp::Time> &
MessageStampRecorderInterface::received_stamps() const
{
  return received_stamps_;
}

} // nvblox
