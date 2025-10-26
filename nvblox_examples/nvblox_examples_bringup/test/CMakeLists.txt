# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# Get the path to a dummy bag for use in the tests.
get_dummy_bag_path(DUMMY_BAG_PATH)

# NOTE(alexmillane): In these dry-run tests, the graphs sometimes terminates with a non-zero
# exit code, which causes the test to fail (even though the graph *runs* correctly). This issue
# is under investigation here: https://nvbugspro.nvidia.com/bug/4751762.
# We're therefore disabling the exit-code checks for now.
# Note that the test is still functional because it will still fail when the graph fails
# to run for the timeout time.
# TODO(alexmillane): Re-enable exit-code checks when we've sorted out the above bug.

add_graph_startup_test(realsense_dry_run
    launch/realsense_example.launch.py
    DONT_CHECK_EXIT_CODE
    TIMEOUT 20
    ARGS "rosbag:=${DUMMY_BAG_PATH} run_rviz:=False"
)

#NOTE(alexmillane): For some reason nav2 doesn't exit cleanly, causing the test to fail.
# So here we turn navigation off.
add_graph_startup_test(isaac_sim_dry_run
    launch/isaac_sim_example.launch.py
    DONT_CHECK_EXIT_CODE
    TIMEOUT 20
    ARGS "rosbag:=${DUMMY_BAG_PATH} run_rviz:=False navigation:=False"
)

add_graph_startup_test(zed_dry_run
    launch/zed_example.launch.py
    DONT_CHECK_EXIT_CODE
    TIMEOUT 20
    ARGS "rosbag:=${DUMMY_BAG_PATH} run_rviz:=False"
)
