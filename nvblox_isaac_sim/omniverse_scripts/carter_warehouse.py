# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
import argparse
import time

import carb
from omni.isaac.kit import SimulationApp

from common.scenario import load_scenario


def meters_to_stage_units(length_m: float) -> float:
    from omni.isaac.core.utils.stage import get_stage_units
    stage_units_in_meters = get_stage_units()
    return length_m / stage_units_in_meters


def stage_units_to_camera_units(length_in_stage_units: float) -> float:
    camera_lengths_in_stage_units = 1.0 / 10.0
    return length_in_stage_units / camera_lengths_in_stage_units


def setup_carter_sensors(
        carter_prim_path: str, camera_focal_length_m: float = 0.009,
        carter_version: int = 1):

    # Set up variables based on carter version.
    left_cam = "ROS_Camera_Stereo_Left"
    right_cam = "ROS_Camera_Stereo_Right"
    left_cam_path = "chassis_link/camera_mount/carter_camera_stereo_left"
    right_cam_path = "chassis_link/camera_mount/carter_camera_stereo_right"
    stereo_offset = -32.985  # TODO: double check this one; -41.23125

    if carter_version == 2:
        left_cam = "ROS_Stereo_Camera_Left"
        right_cam = "ROS_Stereo_Camera_Right"
        left_cam_path = "chassis_link/stereo_cam_left/stereo_cam_left_sensor_frame/camera_sensor_left"
        right_cam_path = "chassis_link/stereo_cam_right/stereo_cam_right_sensor_frame/camera_sensor_right"
        stereo_offset = -32.985  # TODO: double check this one

    import omni
    from pxr import Sdf
    from pxr import Gf
    # Enable
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{right_cam}.enabled"),
        value=True, prev=None)
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{left_cam}.enabled"),
        value=True, prev=None)
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/ROS_DifferentialBase.enabled"),
        value=True, prev=None)
    omni.kit.commands.execute(
        "ChangeProperty",
        prop_path=Sdf.Path(
            f"{carter_prim_path}/ROS_Carter_Sensors_Broadcaster.enabled"),
        value=True,
        prev=None,
    )
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/ROS_Carter_Broadcaster.enabled"),
        value=True, prev=None)
    omni.kit.commands.execute("ChangeProperty", prop_path=Sdf.Path(
        f"{carter_prim_path}/ROS_Clock.enabled"), value=True, prev=None)
    # Use LIDAR for the NAv2 Demo
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/ROS_Lidar.enabled"),
        value=True, prev=None)
    # Enable RBG on right cam, Depth on left cam
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{left_cam}.rgbEnabled"),
        value=True, prev=None)
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{left_cam}.depthEnabled"),
        value=True, prev=None)
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{right_cam}.rgbEnabled"),
        value=True, prev=None)
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{right_cam}.depthEnabled"),
        value=False, prev=None)
    # Change the camera resolution to something less high-def
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{left_cam}.resolution"),
        value=Gf.Vec2i(640, 480),
        prev=None)
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{right_cam}.resolution"),
        value=Gf.Vec2i(640, 480),
        prev=None)
    # Change the stereo offset in mystery units of mysteriousness
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{right_cam}.stereoOffset"),
        value=Gf.Vec2f(stereo_offset, 0.0),
        prev=None)
    # Change the output topics of the camera and stuff
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{left_cam}.cameraInfoPubTopic"),
        value="/left/camera_info",
        prev=None)
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{left_cam}.depthPubTopic"),
        value="/left/depth",
        prev=None)
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{left_cam}.rgbPubTopic"),
        value="/left/rgb",
        prev=None)
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{right_cam}.cameraInfoPubTopic"),
        value="/right/camera_info",
        prev=None)
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{right_cam}.depthPubTopic"),
        value="/right/depth",
        prev=None)
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{right_cam}.rgbPubTopic"),
        value="/right/rgb",
        prev=None)

    # Change the focal length of the camera (default in the carter model quite narrow).
    camera_focal_length_in_camera_units = stage_units_to_camera_units(
        meters_to_stage_units(camera_focal_length_m))

    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{left_cam_path}.focalLength"),
        value=camera_focal_length_in_camera_units, prev=None)
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(
            f"{carter_prim_path}/{right_cam_path}.focalLength"),
        value=camera_focal_length_in_camera_units, prev=None)


def main(
        scenario_path: str, tick_rate_hz: float = 20.0, headless: bool = False,
        carter_prim_path: str = "/World/Carter_ROS",
        carter_version: int = 1):
    # Start up the simulator
    simulation_app = SimulationApp(
        {"renderer": "RayTracedLighting", "headless": headless})
    import omni
    from omni.isaac.core import SimulationContext

    # enable ROS2 bridge extension
    from omni.isaac.core.utils.extensions import enable_extension
    enable_extension("omni.isaac.ros2_bridge")

    # Load the stage
    stage_units_m = load_scenario(simulation_app, scenario_path)
    time_dt = 1.0 / tick_rate_hz
    print(f"Running sim at {tick_rate_hz} Hz, with dt of {time_dt}")
    simulation_context = SimulationContext(
        physics_dt=time_dt, rendering_dt=time_dt,
        stage_units_in_meters=stage_units_m)

    # Configure sensors
    print(
        f"Configuring sensors for Carter {carter_version} at: {carter_prim_path}")
    setup_carter_sensors(carter_prim_path, carter_version=carter_version)
    simulation_context.play()
    simulation_context.step()

    # Tick all of the components once to make sure all of the ROS nodes are initialized
    # For cameras this also handles viewport initialization etc.
    omni.kit.commands.execute("Ros2BridgeUseSimTime",
                              use_sim_time=True)

    # Simulate for one second to warm up sim and let everything settle
    # Otherwise the resizes below sometimes don't stick.
    for frame in range(round(tick_rate_hz)):
       simulation_context.step()

    # Dock the second camera window
    right_viewport = omni.ui.Workspace.get_window("Viewport")
    left_viewport = omni.ui.Workspace.get_window("Viewport 2")
    if right_viewport is not None and left_viewport is not None:
        left_viewport.dock_in(right_viewport, omni.ui.DockPosition.LEFT)
    right_viewport = None
    left_viewport = None

    # Run the sim
    last_frame_time = time.monotonic()
    while simulation_app.is_running():
        simulation_context.step()
        current_frame_time = time.monotonic()
        if current_frame_time - last_frame_time < time_dt:
            time.sleep(time_dt - (current_frame_time - last_frame_time))
        last_frame_time = time.monotonic()

    simulation_context.stop()
    simulation_app.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Sample app for running Carter in a Warehouse for NVblox.")
    parser.add_argument(
        "--scenario_path", metavar="scenario_path", type=str,
        help="Path of the scenario to launch relative to the nucleus server "
        "base path. Scenario must contain a carter robot.",
        default="/Isaac/Samples/ROS/Scenario/carter_warehouse_navigation.usd")
    parser.add_argument("--tick_rate_hz", metavar="tick_rate_hz", type=int,
                        help="The rate (in hz) that we step the simulation at.",
                        default=20)
    parser.add_argument(
        "--carter_prim_path", metavar="carter_prim_path", type=str,
        default="/World/Carter_ROS", help="Path to Carter.")
    parser.add_argument("--headless", action='store_true')
    parser.add_argument(
        "--carter_version", type=int, default=1,
        help="Version of the Carter robot (1 or 2)")
    args, unknown = parser.parse_known_args()

    main(args.scenario_path, args.tick_rate_hz,
         args.headless, args.carter_prim_path, args.carter_version)
