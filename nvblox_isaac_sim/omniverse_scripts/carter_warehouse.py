# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import argparse
import time

from omni.isaac.kit import SimulationApp


def meters_to_stage_units(length_m: float) -> float:
    from omni.isaac.core.utils.stage import get_stage_units
    stage_units_in_meters = get_stage_units()
    return length_m / stage_units_in_meters


def stage_units_to_camera_units(length_in_stage_units: float) -> float:
    camera_lengths_in_stage_units = 1.0 / 10.0
    return length_in_stage_units / camera_lengths_in_stage_units


def setup_carter_sensors(carter_prim_path: str,
                         camera_focal_length_m: float = 0.009,
                         carter_version: int = 1):

    # Set up variables based on carter version.
    left_cam_path = 'chassis_link/camera_mount/carter_camera_stereo_left'
    right_cam_path = 'chassis_link/camera_mount/carter_camera_stereo_right'

    if carter_version == 2:
        left_cam_path = (
            'chassis_link/stereo_cam_left/stereo_cam_left_sensor_frame/'
            'camera_sensor_left')
        right_cam_path = (
            'chassis_link/stereo_cam_right/stereo_cam_right_sensor_frame/'
            'camera_sensor_right')

    import omni
    from pxr import Sdf

    # Enable
    # Change the focal length of the camera (default in the carter model quite narrow).
    camera_focal_length_in_camera_units = stage_units_to_camera_units(
        meters_to_stage_units(camera_focal_length_m))

    omni.kit.commands.execute(
        'ChangeProperty',
        prop_path=Sdf.Path(f'{carter_prim_path}/{left_cam_path}.focalLength'),
        value=camera_focal_length_in_camera_units,
        prev=None)
    omni.kit.commands.execute(
        'ChangeProperty',
        prop_path=Sdf.Path(f'{carter_prim_path}/{right_cam_path}.focalLength'),
        value=camera_focal_length_in_camera_units,
        prev=None)

    # Lidar setup
    carter_lidar_path = 'chassis_link/carter_lidar'

    # Set up Lidar for a 32 layer like configuration
    omni.kit.commands.execute(
        'ChangeProperty',
        prop_path=Sdf.Path(f'{carter_prim_path}/{carter_lidar_path}.highLod'),
        value=True,
        prev=None)
    omni.kit.commands.execute(
        'ChangeProperty',
        prop_path=Sdf.Path(
            f'{carter_prim_path}/{carter_lidar_path}.horizontalResolution'),
        value=0.2,
        prev=None,
    )
    omni.kit.commands.execute(
        'ChangeProperty',
        prop_path=Sdf.Path(f'{carter_prim_path}/{carter_lidar_path}.minRange'),
        value=0.7,
        prev=None,
    )
    omni.kit.commands.execute(
        'ChangeProperty',
        prop_path=Sdf.Path(
            f'{carter_prim_path}/{carter_lidar_path}.verticalFov'),
        value=30.0,
        prev=None,
    )
    omni.kit.commands.execute(
        'ChangeProperty',
        prop_path=Sdf.Path(
            f'{carter_prim_path}/{carter_lidar_path}.verticalResolution'),
        value=1.0,
        prev=None,
    )

    # Modify the omnigraph to get lidar point cloud published

    import omni.graph.core as og

    controller = og.Controller()

    # Create pcl reader node
    read_pcl_node_path = f'{carter_prim_path}/ActionGraph/isaac_read_lidar_point_cloud_node'
    read_pcl_node = controller.create_node(
        read_pcl_node_path,
        'omni.isaac.range_sensor.IsaacReadLidarPointCloud',
        True,
    )
    # Add relationship to lidar prim
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    read_pcl_prim = stage.GetPrimAtPath(read_pcl_node_path)
    input_rel = read_pcl_prim.GetRelationship('inputs:lidarPrim')
    input_rel.SetTargets([f'{carter_prim_path}/{carter_lidar_path}'])
    # Retrieve on playback node
    playback_node = controller.node('on_playback_tick',
                                    f'{carter_prim_path}/ActionGraph')
    # Connect the tick to the lidar pcl read
    controller.connect(playback_node.get_attribute('outputs:tick'),
                       read_pcl_node.get_attribute('inputs:execIn'))

    # Create ros2 publisher node
    publish_pcl_node = controller.create_node(
        f'{carter_prim_path}/ActionGraph/ros2_publish_point_cloud',
        'omni.isaac.ros2_bridge.ROS2PublishPointCloud', True)
    # Set frame id
    publish_pcl_node.get_attribute('inputs:frameId').set('carter_lidar')
    # Connect pcl read to pcl publish
    controller.connect(read_pcl_node.get_attribute('outputs:execOut'),
                       publish_pcl_node.get_attribute('inputs:execIn'))
    controller.connect(read_pcl_node.get_attribute('outputs:pointCloudData'),
                       publish_pcl_node.get_attribute('inputs:pointCloudData'))
    # Get timestamp node and connect it
    timestamp_node = controller.node('isaac_read_simulation_time',
                                     f'{carter_prim_path}/ActionGraph')
    controller.connect(timestamp_node.get_attribute('outputs:simulationTime'),
                       publish_pcl_node.get_attribute('inputs:timeStamp'))
    # Get context node and connect it
    context_node = controller.node('ros2_context',
                                   f'{carter_prim_path}/ActionGraph')
    controller.connect(context_node.get_attribute('outputs:context'),
                       publish_pcl_node.get_attribute('inputs:context'))
    # Get namespace node and connect it
    namespace_node = controller.node('node_namespace',
                                     f'{carter_prim_path}/ActionGraph')
    controller.connect(namespace_node.get_attribute('inputs:value'),
                       publish_pcl_node.get_attribute('inputs:nodeNamespace'))

    # Configure left camera resolution
    # Create camera node and set target resolution
    resolution_node = controller.create_node(
        f'{carter_prim_path}/ROS_Cameras/isaac_set_viewport_resolution',
        'omni.isaac.core_nodes.IsaacSetViewportResolution',
        True,
    )
    resolution_node.get_attribute('inputs:width').set(640)
    resolution_node.get_attribute('inputs:height').set(480)
    # Get viewport creation node
    viewport_node = controller.node('isaac_create_viewport_left',
                                    f'{carter_prim_path}/ROS_Cameras')
    # Connect it
    controller.connect(viewport_node.get_attribute('outputs:execOut'),
                        resolution_node.get_attribute('inputs:execIn'))
    controller.connect(viewport_node.get_attribute('outputs:viewport'),
                        resolution_node.get_attribute('inputs:viewport'))

    # Change publication topics
    left_rgb = controller.node('ros2_create_camera_left_rgb',
                               f'{carter_prim_path}/ROS_Cameras')
    left_rgb.get_attribute('inputs:topicName').set('/left/rgb')
    left_info = controller.node('ros2_create_camera_left_info',
                                f'{carter_prim_path}/ROS_Cameras')
    left_info.get_attribute('inputs:topicName').set('/left/camera_info')
    left_depth = controller.node('ros2_create_camera_left_depth',
                                 f'{carter_prim_path}/ROS_Cameras')
    left_depth.get_attribute('inputs:topicName').set('/left/depth')

    # Finally, enable left rgb and depth
    for enable_name in [
            'enable_camera_left', 'enable_camera_left_rgb',
            'enable_camera_left_depth'
    ]:
        left_enable = controller.node(enable_name,
                                      f'{carter_prim_path}/ROS_Cameras')
        left_enable.get_attribute('inputs:condition').set(True)


def setup_forklifts_collision():
    # Finally forklifts

    import omni.kit.commands
    from omni.isaac.core.utils.prims import is_prim_path_valid
    from pxr import Sdf

    for forklift_name in ['Forklift', 'Forklift_01']:
        forklift_path = f'/World/warehouse_with_forklifts/{forklift_name}'
        # Ignore if they do not exist
        if not is_prim_path_valid(forklift_path):
            continue
        # Enable collision on main body
        omni.kit.commands.execute(
            'ChangeProperty',
            prop_path=Sdf.Path(
                f'{forklift_path}/S_ForkliftBody.physics:collisionEnabled'),
            value=True,
            prev=None,
        )
        # Disable collision on invible main body box
        omni.kit.commands.execute(
            'ChangeProperty',
            prop_path=Sdf.Path(
                f'{forklift_path}/chassis_collision.physics:collisionEnabled'),
            value=False,
            prev=None,
        )
        # Enable collision on tine
        omni.kit.commands.execute(
            'ChangeProperty',
            prop_path=Sdf.Path(
                f'{forklift_path}/S_ForkliftFork/collision_box.physics:collisionEnabled'
            ),
            value=False,
            prev=None,
        )
        # Disable collision on tine box
        omni.kit.commands.execute(
            'ChangeProperty',
            prop_path=Sdf.Path(
                f'{forklift_path}/S_ForkliftFork/S_ForkliftFork.physics:collisionEnabled'
            ),
            value=True,
            prev=None,
        )


def main(scenario_path: str,
         tick_rate_hz: float = 20.0,
         headless: bool = False,
         carter_prim_path: str = '/World/Carter_ROS',
         carter_version: int = 1):
    # Start up the simulator
    simulation_app = SimulationApp({
        'renderer': 'RayTracedLighting',
        'headless': headless
    })
    import omni
    from omni.isaac.core import SimulationContext
    # enable ROS2 bridge extension
    from omni.isaac.core.utils.extensions import enable_extension
    enable_extension('omni.isaac.ros2_bridge')

    from omni.isaac.core.utils.nucleus import get_assets_root_path

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        import carb
        carb.log_error('Could not find Isaac Sim assets folder')
        simulation_app.close()
        exit()

    usd_path = assets_root_path + scenario_path
    print('------{0} n scenario_path:{1}', usd_path, scenario_path)

    # Load the stage

    omni.usd.get_context().open_stage(usd_path, None)

    # Wait two frames so that stage starts loading
    simulation_app.update()
    simulation_app.update()

    print('Loading stage...')
    from omni.isaac.core.utils.stage import is_stage_loading

    while is_stage_loading():
        simulation_app.update()
    print('Loading Complete')

    time_dt = 1.0 / tick_rate_hz
    print(f'Running sim at {tick_rate_hz} Hz, with dt of {time_dt}')
    simulation_context = SimulationContext(stage_units_in_meters=1.0)

    # Configure sensors
    print(
        f'Configuring sensors for Carter {carter_version} at: {carter_prim_path}'
    )
    setup_carter_sensors(carter_prim_path, carter_version=carter_version)
    setup_forklifts_collision()

    simulation_context.play()
    simulation_context.step()

    # Simulate for one second to warm up sim and let everything settle
    # Otherwise the resizes below sometimes don't stick.
    for frame in range(round(tick_rate_hz)):
        simulation_context.step()

    # Dock the second camera window
    right_viewport = omni.ui.Workspace.get_window('Viewport')
    left_viewport = omni.ui.Workspace.get_window('Viewport 2')
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
        description='Sample app for running Carter in a Warehouse for NVblox.')
    parser.add_argument(
        '--scenario_path',
        metavar='scenario_path',
        type=str,
        help='Path of the scenario to launch relative to the nucleus server '
        'base path. Scenario must contain a carter robot.',
        default='/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd')
    parser.add_argument(
        '--tick_rate_hz',
        metavar='tick_rate_hz',
        type=int,
        help='The rate (in hz) that we step the simulation at.',
        default=20)
    parser.add_argument('--carter_prim_path',
                        metavar='carter_prim_path',
                        type=str,
                        default='/World/Carter_ROS',
                        help='Path to Carter.')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--carter_version',
                        type=int,
                        default=1,
                        help='Version of the Carter robot (1 or 2)')
    args, unknown = parser.parse_known_args()

    main(args.scenario_path, args.tick_rate_hz, args.headless,
         args.carter_prim_path, args.carter_version)
