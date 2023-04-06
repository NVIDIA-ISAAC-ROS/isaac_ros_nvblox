# Tutorial For Human Reconstruction in Isaac Sim

<div align="center"><img src="../resources/isaac_sim_nvblox_humans.gif"/></div>

In this tutorial we'll demonstrate how one can perform human reconstruction in nvblox with Isaac Sim. 
The detected human is then visible in a separate dynamic costmap that can be used for navigation with Nav2.
If you want to know how human reconstruction works behind the scenes refer to the [technical details](./technical-details.md#human-reconstruction).

Before continuing this tutorial, make sure that the [Isaac Sim](tutorial-isaac-sim.md) tutorial is working fine.

## Isaac Sim Human Reconstruction

In this part, we demonstrate how to use Isaac Sim for human reconstruction in nvblox. We use the
ground truth semantic segmentation coming from the simulation as input to detect humans. This
demonstration relies on the extensions [*omni.anim.people*](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_anim_people.html) and [*omni.anim.navigation*](https://docs.omniverse.nvidia.com/prod_extensions/prod_extensions/ext_navigation-mesh.html) to make
humans navigate in the environment while the robot is moving.

### First time setup

Complete steps 1-4 in the [Isaac Sim](tutorial-isaac-sim.md) tutorial to set up Isaac Sim, build the ROS container and the workspace.

  > **Note**: In a linux based system the usd file, behavior scripts, and the command file have to be on the same file system. If you are not using our sample usd scene, you need to apply [this workaround](linux-people-isaac-sim-workaround.md).


### Running the demonstration scene

1. Run the ROS Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

2. Start the simulation by running the below command in a terminal **outside the Docker container**:

    ```bash
    alias omni_python='~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh' && \
      omni_python ~/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_isaac_sim/omniverse_scripts/start_isaac_sim.py --with_people --scenario_path=/Isaac/Samples/NvBlox/carter_warehouse_navigation_with_people.usd
    ```

    > **Note**: The `start_isaac_sim` script is explained in detail [here](../nvblox_examples/nvblox_isaac_sim/omniverse_scripts/README.md).

    > **Note**: If you are using a different version of Isaac Sim, replace `isaac_sim-2022.2.1` with your version found at `'~/.local/share/ov/pkg/`

    > **Note**: We strongly advise using `isaac_sim-2022.2.1` because our code is tested and requires some extensions introduced on this version.

    > **Note**: Because the animation requires to execute Python scripts, running the scene with the UI will pop a window up asking to confirm that you want to enable script execution. Click Yes to make it possible to start the scene and the human animation.

     <div align="center"><img src="../resources/animation_confirmation_dialog.png" width=800px/></div>

3. Launch nvblox configured for human mapping:

    ```bash
    ros2 launch nvblox_examples_bringup isaac_sim_humans_example.launch.py
    ```

Step 2 launches the default human paths. If you want to have other paths taken by the humans, you can either:

- change the default human animation file manually directly on the server
- use the randomization options of the above script. To do so:

  ```bash
    alias omni_python='~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh' && \
      omni_python ~/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_isaac_sim/omniverse_scripts/start_isaac_sim.py \
      --with_people --scenario_path=/Isaac/Samples/NvBlox/carter_warehouse_navigation_with_people.usd \
      --anim_people_waypoint_dir <path_to_folder> --with_people --random_command_generation
  ```

  This will launch the scene headless (without visualization) and generate a new `human_cmd_file.txt` in `<path_to_folder>`. By default, it will generate 5 waypoints per human, but it is possible to change this with the option `--num_waypoints=<number_of_waypoints>`. Then you can either manually upload the script file to replace the default one, or use the same command as step 2. by adding `--use_generated_command_file --anim_people_waypoint_dir <path_to_folder>` to automatically set the script file.

### Running on a custom scene

If you want to test the reconstruction on another scene:

* Follow the steps from the [setup section](#first-time-setup) to have the scripts available.

* Make sure you use the same robot usd so that the topic names and ROS publication graphs are correctly set up.

* Make sure that humans that you add to the scene have the `person` semantic segmentation class. To do so, you can use the Semantics Schema Editor on the top prim of the additional humans.

* Make sure that all humans are under */World/Humans* for them to be picked up by the randomization.
