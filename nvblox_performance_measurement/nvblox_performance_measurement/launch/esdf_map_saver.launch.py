# Copyright (c) 2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    output_file_name_arg = DeclareLaunchArgument(
        "file_name",
        default_value="output.png",
        description="The output filename")

    folder_name_arg = DeclareLaunchArgument(
        "folder_path",
        default_value="/tmp",
        description="The folder to save the output file",
    )

    pc_sub = Node(
        package="nvblox_performance_measurement",
        executable="esdf_map_saver.py",
        name="esdf_map_saver",
        output="screen",
        parameters=[{
            "file_name": LaunchConfiguration("file_name"),
            "folder_path": LaunchConfiguration("folder_path"),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([output_file_name_arg, folder_name_arg, pc_sub])
