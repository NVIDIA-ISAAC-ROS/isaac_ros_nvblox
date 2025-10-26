#!/bin/bash
# Copyright (c) 2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Download the example data for the nova_carter repo. We cache the downloaded files. To
# force a download delete the downloaded files first.

set -e

#NOTE(alexmillane): Right now were using artifactory, which is a NVIDIA private link.
#                   Need to update this to a public NGC link once we go live.

[[ -z "$ISAAC_ROS_WS" ]] && echo "ERROR: ISAAC_ROS_WS is not set." && exit 1

EXAMPLE_BAG_NAME="2024_09_23_nvblox_quickstart_galileo_static_3_2"
ARTIFACTORY_URL="https://urm.nvidia.com/artifactory/sw-isaac-sdk-generic-local/dependencies/internal/data"
ARCHIVE_FILENAME="${EXAMPLE_BAG_NAME}.tar.gz"
EXAMPLE_BAG_URL="${ARTIFACTORY_URL}/${ARCHIVE_FILENAME}"

ROSBAG_ASSET_DIR="${ISAAC_ROS_WS}/isaac_ros_assets/rosbags"
ASSET_INSTALL_PATHS="${ROSBAG_ASSET_DIR}/${EXAMPLE_BAG_NAME}/metadata.yaml ${ROSBAG_ASSET_DIR}/${EXAMPLE_BAG_NAME}/galileo_static_3_2_0.mcap"
DOWNLOAD_PATH="/tmp/${ARCHIVE_FILENAME}"

[[ $1 == "--print-install-paths" ]] && echo -n "$ASSET_INSTALL_PATHS" && exit 0

set -x
mkdir -p "${ROSBAG_ASSET_DIR}"

echo "Downloading test bag: $EXAMPLE_BAG_NAME\nfrom:$EXAMPLE_BAG_URL"
wget --no-check-certificate "${EXAMPLE_BAG_URL}" -O "${DOWNLOAD_PATH}"
tar -xzvf "${DOWNLOAD_PATH}" -C "${ROSBAG_ASSET_DIR}"

rm -f "${DOWNLOAD_PATH}"
