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

import argparse
import os
import random
import time

import omni

ADDITIONAL_EXTENSIONS_BASE = ['omni.isaac.ros2_bridge']

ADDITIONAL_EXTENSIONS_PEOPLE = [
    'omni.anim.people', 'omni.anim.navigation.bundle', 'omni.anim.timeline',
    'omni.anim.graph.bundle', 'omni.anim.graph.core', 'omni.anim.graph.ui',
    'omni.anim.retarget.bundle', 'omni.anim.retarget.core',
    'omni.anim.retarget.ui', 'omni.kit.scripting']


def enable_extensions_for_sim(with_people: bool = False):
    """
    Enable the required extensions for the simulation.

    Args:
        with_people (bool, optional): Loads the human animation extensions if the scene with
                                      humans is requested. Defaults to False.

    """
    from omni.isaac.core.utils.extensions import enable_extension
    add_exten = ADDITIONAL_EXTENSIONS_BASE
    if with_people:
        add_exten = ADDITIONAL_EXTENSIONS_BASE + ADDITIONAL_EXTENSIONS_PEOPLE
    for idx in range(len(add_exten)):
        enable_extension(add_exten[idx])


def rebuild_nav_mesh():
    """
    Rebuild the navmesh with the correct settings. Used for the people to move around.

    Called only when the sim with people is requested.
    """
    # Navigation mesh rebake settings
    import omni.kit.commands
    print('Rebuilding navigation mesh...')

    omni.kit.commands.execute(
        'ChangeSetting',
        path='/exts/omni.anim.navigation.core/navMesh/config/agentHeight',
        value=1.5)
    omni.kit.commands.execute(
        'ChangeSetting',
        path='/exts/omni.anim.navigation.core/navMesh/config/agentRadius',
        value=0.5)
    omni.kit.commands.execute(
        'ChangeSetting',
        path='/persistent/exts/omni.anim.navigation.core/navMesh/autoRebakeDelaySeconds',
        value=4)
    omni.kit.commands.execute(
        'ChangeSetting',
        path='/persistent/exts/omni.anim.navigation.core/navMesh/viewNavMesh',
        value=False)
    print('Navigation mesh rebuilt.')


def create_people_commands(environment_prim_path: str,
                           anim_people_command_dir: str,
                           max_number_tries: int,
                           num_waypoints_per_human: int):
    """
    Create the occupancy grid which returns the free points for the humans to move.

    These points are then randomly sampled and assigned to each person and is saved as a txt file
    which can be used as the command file for people animation. It directly gets the context of
    the scene once the scene is opened and played in IsaacSim.

    Args:
        environment_prim_path (str): The warehouse enviroment prim path
        anim_people_command_dir (str): The local directory to be used for storing the command file
        max_number_tries (int): The number of times the script attempts to select a new waypoint
        for a human.
        num_waypoints_per_human (int): The number of waypoints to create per human.

    """
    import carb
    import omni.anim.navigation.core as navcore
    from omni.isaac.core.prims import XFormPrim
    from omni.isaac.occupancy_map import _occupancy_map
    navigation_interface = navcore.acquire_interface()
    print('Creating randomized paths for people...')
    bounding_box = omni.usd.get_context().compute_path_world_bounding_box(
        environment_prim_path)

    physx = omni.physx.acquire_physx_interface()
    stage_id = omni.usd.get_context().get_stage_id()

    generator = _occupancy_map.Generator(physx, stage_id)
    # 0.05m cell size, output buffer will have 0 for occupied cells, 1 for
    # unoccupied, and 6 for cells that cannot be seen
    # this assumes your usd stage units are in m, and not cm
    generator.update_settings(.05, 0, 1, 6)
    # Setting the transform for the generator
    generator.set_transform(
        # The starting location to map from
        (0, 0, 0.025),
        # The min bound
        (bounding_box[0][0], bounding_box[0][1], 0.025),
        # The max bound
        (bounding_box[1][0], bounding_box[1][1], 0.025))
    generator.generate2d()
    # Get locations of free points
    free_points = generator.get_free_positions()
    # Find out number of humans in scene and create a list with their names
    human_names = []
    human_prims = []
    for human in omni.usd.get_context().get_stage().GetPrimAtPath(
            '/World/Humans').GetAllChildren():
        human_name = human.GetChildren()[0].GetName()
        # To remove biped setup prim from the list
        if 'CharacterAnimation' in human_name or 'Biped_Setup' in human_name:
            continue
        human_names.append(human_name)
        human_prims.append(XFormPrim(human.GetPath().pathString))

    random_waypoints = {}
    for human_name, human_prim in zip(human_names, human_prims):
        # Get the human initial position.
        start_point, _ = human_prim.get_world_pose()
        # Initialize target waypoints.
        random_waypoints[human_name] = []
        for _ in range(num_waypoints_per_human):
            current_tries = 0
            path = None
            # Sample for a maximum number of tries then give up.
            while path is None and current_tries < max_number_tries:
                new_waypoint = random.sample(free_points, 1)
                # Check that a path exists between the starting position and the destination
                path = navigation_interface.query_navmesh_path(
                    carb.Float3(start_point), new_waypoint[0])
                current_tries += 1
            if path is not None:
                new_waypoint[0].z = 0.0
                random_waypoints[human_name].append(new_waypoint[0])
                print(
                    f'Found path for {human_name} from {start_point} to {new_waypoint[0]} after \
                     {current_tries} tries')
                start_point = new_waypoint[0]
            else:
                print(
                    f'Could not find path for {human_name} after {max_number_tries}, skipping')

    # Save as command file
    command_file_path = os.path.join(
        anim_people_command_dir, 'human_cmd_file.txt')
    print(f'Saving randomized commands to {command_file_path}')
    with open(command_file_path, 'w') as file:
        for human_name, waypoints in random_waypoints.items():
            human_command_line = f'{human_name} GoTo '
            for next_waypoint in waypoints:
                human_command_line += f'{next_waypoint[0]} {next_waypoint[1]} 0 '
            human_command_line += '_\n'
            file.write(human_command_line)


def update_people_command_file_path(anim_people_command_dir: str):
    """
    Update the command file path settings in the simulation scene to the custom one.

    Args:
        anim_people_command_dir (str): The directory where the generated/custom command file is
                                       stored.

    """
    import omni.kit.commands
    omni.kit.commands.execute(
        'ChangeSetting',
        path='/exts/omni.anim.people/command_settings/command_file_path',
        value=anim_people_command_dir + '/human_cmd_file.txt')


def main(scenario_path: str,
         anim_people_command_dir: str,
         environment_prim_path: str,
         physics_scene_path: str,
         random_command_generation: bool,
         num_waypoints: int,
         headless: bool = False,
         with_people: bool = True,
         use_generated_command_file: bool = False,
         gpu_physics_enabled: bool = False,
         tick_rate_hz: float = 20.0):

    # Start up the simulator
    from omni.isaac.kit import SimulationApp
    simulation_app = SimulationApp({
        'renderer': 'RayTracedLighting',
        'headless': headless
    })

    import omni.kit.commands
    from omni.isaac.core import SimulationContext
    from omni.isaac.core.utils.nucleus import get_assets_root_path

    # Enables the simulation extensions
    enable_extensions_for_sim(with_people)

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        print(
            'Could not find Isaac Sim assets folder. Make sure you have an up to date local \
             Nucleus server or that you have a proper connection to the internet.'
        )
        simulation_app.close()
        exit()

    usd_path = assets_root_path + scenario_path
    print(f'Opening stage {usd_path}...')

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

    # If the scene with people is requested we build/rebuild the navmesh
    # We also point to the generated/custom command file for people animation if set by user
    if with_people:
        rebuild_nav_mesh()
        if not random_command_generation and use_generated_command_file:
            update_people_command_file_path(anim_people_command_dir)

    time_dt = 1.0 / tick_rate_hz
    print(f'Running sim at {tick_rate_hz} Hz, with dt of {time_dt}')
    # Run physics at 60 Hz and render time at the set frequency to see the sim as real time
    simulation_context = SimulationContext(stage_units_in_meters=1.0,
                                           physics_dt=1.0 / 60,
                                           rendering_dt=time_dt)

    if gpu_physics_enabled:
        from pxr import Sdf
        omni.kit.commands.execute('ChangeProperty',
                                  prop_path=Sdf.Path(
                                      physics_scene_path + '.physxScene:enableGPUDynamics'),
                                  value=True,
                                  prev=None)

    simulation_context.play()
    simulation_context.step()

    # Physics can only be seen when the scene is played
    # If we want to generate the command file for the people simulation we call the
    # create_people_commands method, which stores the command file and then we close the
    # simulation and point the user on how to use the generated file
    if with_people and random_command_generation:
        print('Creating human animation file...')
        create_people_commands(environment_prim_path,
                               anim_people_command_dir, 10, num_waypoints)
        print(
            'Human command file has been created at {}/human_human_cmd_file.txt'
            .format(anim_people_command_dir))
        print(
            'Please restart the simulation with --with_people and \n \
              --use_generated_command_file to use the generated command file in human simulation'
        )
        simulation_context.stop()
        simulation_app.close()

    # Simulate for a few seconds to warm up sim and let everything settle
    for _ in range(2*round(tick_rate_hz)):
        simulation_context.step()

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
        help='Path of the scenario to launch relative to the nucleus server '
        'base path. Scenario must contain a carter robot. If the scene '
        'contains animated humans, the script expects to find them under /World/Humans. '
        'Example scenarios are /Isaac/Samples/NvBlox/carter_warehouse_navigation_with_people.usd '
        'or /Isaac/Samples/NvBlox/carter_warehouse_navigation.usd',
        default='/Isaac/Samples/NvBlox/carter_warehouse_navigation.usd')
    parser.add_argument('--environment_prim_path',
                        default='/World/WareHouse',
                        help='Path to the world to create a navigation mesh.')
    parser.add_argument(
        '--tick_rate_hz',
        type=int,
        help='The rate (in hz) that we step the simulation at.',
        default=20)
    parser.add_argument(
        '--anim_people_waypoint_dir',
        help='Directory location to save the waypoints in a yaml file')
    parser.add_argument('--headless', action='store_true',
                        help='Run the simulation headless.')
    parser.add_argument(
        '--with_people',
        help='If used, animation extensions for humans will be enabled.',
        action='store_true')
    parser.add_argument(
        '--random_command_generation',
        help='Choose whether we generate random waypoint or run sim',
        action='store_true')
    parser.add_argument('--num_waypoints', type=int,
                        help='Number of waypoints to generate for each human in the scene.',
                        default=5)
    parser.add_argument(
        '--use_generated_command_file',
        help='Choose whether to use generated/custom command file or to use the default one to run\
              the people animation',
        action='store_true')
    parser.add_argument(
        '--gpu_physics_enabled',
        help='If used, gpu physics for the scene will be enabled. '
             'To be used in case of deformable bodies',
        action='store_true')
    parser.add_argument('--physics_scene_path',
                        default='/World/PhysicsScene',
                        help='Path to the physics scene')
    # This allows for IsaacSim options to be passed on the SimulationApp.
    args, unknown = parser.parse_known_args()

    # If we want to generate the command file then we run the simulation headless
    if args.random_command_generation:
        args.headless = True

    # Check if the command file directory is given if it has to be generated or used for the
    # simulation
    if args.random_command_generation or args.use_generated_command_file:
        if not args.anim_people_waypoint_dir:
            raise ValueError(
                'Input to command file directory required if custom command file has to be \
                    generated/used!!'
            )

    main(args.scenario_path, args.anim_people_waypoint_dir, args.environment_prim_path,
         args.physics_scene_path, args.random_command_generation,
         args.num_waypoints, args.headless, args.with_people,
         args.use_generated_command_file, args.gpu_physics_enabled, args.tick_rate_hz)
