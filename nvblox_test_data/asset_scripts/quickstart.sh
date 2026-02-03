#!/bin/bash
# Copyright (c) 2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Download the test data from NGC. We cache the downloaded files.
# To force a download delete the downloaded files first.

set -e

[[ -z "$ISAAC_ROS_WS" ]] && echo "ERROR: ISAAC_ROS_WS is not set." && exit 1

# The dataset name
EXAMPLE_BAG_NAME="quickstart"
ARCHIVE_FILENAME="${EXAMPLE_BAG_NAME}.tar.gz"

# NOTE: This is the same path that the isaac_ros_nvblox quickstart is downloaded to.
ROSBAG_ASSET_DIR="${ISAAC_ROS_WS}/isaac_ros_assets"
ASSET_INSTALL_PATHS="${ROSBAG_ASSET_DIR}/isaac_ros_nvblox/${EXAMPLE_BAG_NAME}/metadata.yaml ${ROSBAG_ASSET_DIR}/isaac_ros_nvblox/${EXAMPLE_BAG_NAME}/rosbag2_2024_04_04-15_44_33_0.db3"
DOWNLOAD_PATH="/tmp/${ARCHIVE_FILENAME}"

[[ $1 == "--print-install-paths" ]] && echo -n "$ASSET_INSTALL_PATHS" && exit 0

# NGC download specs. Same as in our quickstart
# https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#download-quickstart-assets
NGC_ORG="nvidia"
NGC_TEAM="isaac"
NGC_RESOURCE="isaac_ros_nvblox_assets"
NGC_VERSION="4.0.0"
NGC_FILENAME=$ARCHIVE_FILENAME
REQ_URL="https://api.ngc.nvidia.com/v2/resources/org/$NGC_ORG/team/$NGC_TEAM/$NGC_RESOURCE/$NGC_VERSION/files?redirect=true&path=$NGC_FILENAME"

set -x
mkdir -p "${ROSBAG_ASSET_DIR}"

echo "Downloading test bag: $EXAMPLE_BAG_NAME"
echo "From: $REQ_URL"
curl -L "${REQ_URL}" -o "${DOWNLOAD_PATH}"
tar -xf "${DOWNLOAD_PATH}" -C "${ROSBAG_ASSET_DIR}"
rm -f "${DOWNLOAD_PATH}"
