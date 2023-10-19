#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import numpy as np
import rclpy
from cv_bridge import \
    CvBridge  # Package to convert between ROS and OpenCV Images
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class MeanIoU(Node):
    def __init__(self) -> None:
        super().__init__('network_performance_node')

        # Publisher
        self.iou_publisher = self.create_publisher(Float32, '~/iou', 10)

        gt_subscriber = Subscriber(
            self, Image, '/semantic_conversion/front/stereo_camera/left/semantic_mono8')
        pred_subscriber = Subscriber(
            self, Image, '/unet/raw_segmentation_mask_depadded', qos_profile=QoSPresetProfiles.SENSOR_DATA.value)

        QueueSize = 5
        TimeGap = 0.1

        self.approximate_synchronizer = ApproximateTimeSynchronizer(
            [gt_subscriber, pred_subscriber],
            QueueSize,
            TimeGap)
        self.approximate_synchronizer.registerCallback(self.iou)

        self.cv_bridge = CvBridge()
        self.SMOOTH = 1e-6

    def iou(self, gt_sem, pred_sem):
        gt = np.asarray(self.cv_bridge.imgmsg_to_cv2(gt_sem))
        pred = np.asarray(self.cv_bridge.imgmsg_to_cv2(pred_sem))
        intersection = (pred & gt).sum()
        union = (pred | gt).sum()
        # Using smooth to prevent division by a zero incase the images are empty
        iou = (intersection + self.SMOOTH) / (union + self.SMOOTH)
        msg = Float32()
        msg.data = iou
        self.iou_publisher.publish(msg)


def main():
    rclpy.init()
    network_performance = MeanIoU()
    rclpy.spin(network_performance)
    network_performance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
