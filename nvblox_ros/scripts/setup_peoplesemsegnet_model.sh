#!/bin/bash
# Copyright (c) 2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Download the weights for ESS. We cache the downloaded files. To
# force a download delete the downloaded files first.

set -e

# Input arguments
[[ ! $# == 2 ]] && echo "Usage: $0 ETLT_FILE_PATH MODEL_TYPE" && exit
etlt_file_path=${1:?}
model_type=${2:?}
model_directory=$(dirname "$etlt_file_path")

# Configuration depending on model type [vanilla, shuffleseg]
if [ ${model_type} = "vanilla" ]; then
    input_name="input_1:0"
elif [ ${model_type} = "shuffleseg" ]; then
    input_name="input_2:0"
else
  echo "Error: Model type name ${model_type} not known. Must be vanilla or shuffleseg." >&2
  exit 1
fi

# Check if etlt file if found
mkdir -p ${model_directory}/1
if [ ! -f ${etlt_file_path} ]; then
  echo "Error: PeopleSemSegnet etlt file not found at path ${etlt_file_path}." >&2
  exit 1
fi

# Convert etlt file
if [ ! -f ${model_directory}/1/model.plan ]; then
  echo "Converting PeopleSemSegnet etlt file to plan file."
  /opt/nvidia/tao/tao-converter \
    -k tlt_encode \
    -d 3,544,960 \
    -p ${input_name},1x3x544x960,1x3x544x960,1x3x544x960 \
    -t fp16 \
    -e ${model_directory}/1/model.plan \
    -o argmax_1 \
    ${etlt_file_path}
fi

# Create config file
config_file_text=$(
  cat <<EOF
name: "$(basename "$model_directory")"
platform: "tensorrt_plan"
max_batch_size: 0
input [
  {
    name: "${input_name}"
    data_type: TYPE_FP32
    dims: [ 1, 3, 544, 960 ]
  }
]
output [
  {
    name: "argmax_1"
    data_type: TYPE_INT32
    dims: [ 1, 544, 960, 1 ]
  }
]
version_policy: {
  specific {
    versions: [ 1 ]
  }
}
EOF
)
echo "$config_file_text" >${model_directory}/config.pbtxt
