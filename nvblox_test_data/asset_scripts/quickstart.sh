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

if [ -z "$ISAAC_ROS_WS" ] && [ -n "$METADATA_PATH" ]; then
  ISAAC_ROS_WS="$(readlink -f "$(dirname "${METADATA_PATH}")/../../..")"
elif [ -z "$ISAAC_ROS_WS" ]; then
  echo "ERROR: ISAAC_ROS_WS is not set." && exit 1
fi

# The dataset name
ASSET_NAME="quickstart"
EULA_URL="https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/resources/isaac_ros_nvblox_assets"
ASSET_DIR="${ISAAC_ROS_WS}/isaac_ros_assets"
ASSET_INSTALL_PATHS="${ASSET_DIR}/isaac_ros_nvblox/${ASSET_NAME}/metadata.yaml ${ASSET_DIR}/isaac_ros_nvblox/${ASSET_NAME}/rosbag2_2024_04_04-15_44_33_0.db3"
ARCHIVE_FILENAME="${ASSET_NAME}.tar.gz"

DOWNLOAD_PATH="${ASSET_DIR}/${ARCHIVE_FILENAME}"

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

rm -rf "${ASSET_DIR}/isaac_ros_nvblox"
mkdir -p "${ASSET_DIR}/isaac_ros_nvblox/${ASSET_NAME}"

if [[ -n "${ISAAC_ROS_ASSETS_TEST}" ]]; then
  if [[ -z "${ISAAC_ROS_NVBLOX_TEST_DATA}" || ! -f "${ISAAC_ROS_NVBLOX_TEST_DATA}" ]]; then
    echo "ERROR: Cache path ${ISAAC_ROS_NVBLOX_TEST_DATA} does not exist."
    exit 1
  fi
  exit 0
fi

if [[ -n "${ISAAC_ROS_NVBLOX_TEST_DATA}" && -f "${ISAAC_ROS_NVBLOX_TEST_DATA}" ]]; then
  echo "Copying artifact from ${ISAAC_ROS_NVBLOX_TEST_DATA} to ${DOWNLOAD_PATH}."
  cp "${ISAAC_ROS_NVBLOX_TEST_DATA}" "${DOWNLOAD_PATH}"
else
  echo "Downloading test bag: $ASSET_NAME from ${REQ_URL}"
  curl -L "${REQ_URL}" -o "${DOWNLOAD_PATH}"
fi

tar -xvf "${DOWNLOAD_PATH}" -C "${ASSET_DIR}"
rm -f "${DOWNLOAD_PATH}"
