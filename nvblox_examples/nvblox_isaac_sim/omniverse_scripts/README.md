# Explanation of the start_isaac_sim file to open and play Isaac Sim examples

start_isaac_sim has mainly three modes which can be used in start_isaac_sim.py

- Mode 1
  - `with_people` : **False**
  - `gpu_physics_enabled`: **True**
  - `random_command_generation` : **False**
  - `use_generated_command_file` : **False** <br>
      Launch isaac sim example with no humans in scene
- Mode 2
  - `with_people` : **True**
  - `gpu_physics_enabled`: **True**
  - `random_command_generation` : **False**
  - `use_generated_command_file` : **False/True** <br>
      Launch isaac sim example with humans in scene. Either use default command file for humans with argument use_generated_command_file as **FALSE** or a custom command file with the argument as **TRUE**. If custom command file is to be used the argument for anim_people_waypoint_dir must be set.
- Mode 3
  - `with_people` : **True**
  - `gpu_physics_enabled`: **True**
  - `random_command_generation` : **True**
  - `use_generated_command_file` : **False** <br>
      Launch isaac sim headless and generate custom command file for human animation. Argument anim_people_waypoint_dir has to be set. The script generated this command file in the specified directory and then closes the simulation. This command file can be used the next time the script is used to run the example with humans in scene.

#### **Parameters Explained**

| Parameter           | Default                                                                                                        | Description                                                                                                                                                                                                |
|---------------------|------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `scenario_path`        | */Isaac/Samples/NvBlox/<br>carter_warehouse_navigation.usd*| Path of the scenario to launch relative to the nucleus server base path. Scenario must contain a carter robot. If the scene contains animated humans, the script expects to find them under /World/Humans.|
| `environment_prim_path` | */World/WareHouse* | Path to the world to create a navigation mesh.|
| `tick_rate_hz` | *20* | The rate (in hz) that we step the simulation at.|
| `anim_people_waypoint_dir` | | Directory location to save the people animation commands as a text file|
| `with_people` | *False* | To choose whether to have scene with people or not |
| `random_command_generation` | *False* | Choose whether we generate random waypoint or run sim |
| `num_waypoints` | *5* | Number of waypoints to generate for each human in the scene |
| `use_generated_command_file` | *False* | Choose whether to use generated/custom command file or to use the default one to run the people animation |
| `gpu_physics_enabled` | *False* | If used, gpu physics for the scene will be enabled. To be used in case of deformable bodies and for stable carter behavior |
| `physics_scene_path` | */World/PhysicsScene* | Path to the physics scene |
