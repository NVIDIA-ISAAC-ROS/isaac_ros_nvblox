# Tutorial for NVBLOX with Isaac Sim

<div align="center"><img src="../resources/isaac_sim_nvblox_nav2.gif"/></div>

## Overview

This tutorial walks you through generating a 3D reconstruction with nvblox using image and LiDAR data from Isaac Sim. The reconstruction is displayed as a 3D mesh in RVIZ. Nvblox also converts the reconstruction into a 2D costmap, which is fed into the Nav2 stack for path planning.

Last validated with [Isaac Sim 2022.1.0](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/release_notes.html#id10).

## Tutorial Walkthrough

1. Complete the [quickstart section](../README.md#quickstart) in the main README.
2. Launch the Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

3. Inside the container, build and source the workspace:

    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

4. Install Isaac Sim following the steps in the [Isaac ROS Isaac Sim Setup Guide](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/isaac-sim-sil-setup.md)
5. Start the simulation by running the below command in a terminal **outside the Docker container**:
The start_isaac_sim script is explained in detail [here](../nvblox_examples/nvblox_isaac_sim/omniverse_scripts/README.md).

    ```bash
    alias omni_python='~/.local/share/ov/pkg/isaac_sim-2022.2.1-rc.3/python.sh' && \
      omni_python ~/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_isaac_sim/omniverse_scripts/start_isaac_sim.py
    ```

    > **Note**: If you are using a different version of Isaac Sim, replace `isaac_sim-2022.2.1-rc.3` with your version found at `'~/.local/share/ov/pkg/`.

6. Launch the pre-composed graph launchfile:

    ```bash
    ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py
    ```

7. Click the **2D Goal Pose** button. You should see the mesh and costmap, as well as the robot moving towards the goal location, as shown at the top of this page.
