# Tutorial with Isaac Sim
> **Note: Isaac Sim 2022.1.0 published on 6/3/2022 does not support ROS2 Humble. Please follow one of the [workarounds](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/isaac-sim-sil-setup.md#isaac-sim-202210-workarounds) before continuing with the tutorial**
1. Complete the [Quickstart section](../README.md#quickstart) in the main README.
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
    ```bash 
    alias omni_python='~/.local/share/ov/pkg/isaac_sim-2022.1.0/python.sh' && \
      omni_python ~/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_isaac_sim/omniverse_scripts/carter_warehouse.py
    ```
> **Note:** If you are using a different version of Isaac Sim, replace `isaac_sim-2022.1.0` with your version found at `'~/.local/share/ov/pkg/`

6. Launch the pre-composed pipeline launchfile: 
    ```bash 
    ros2 launch nvblox_nav2 carter_sim.launch.py
    ```
7. Click on the `2D Goal Pose` button. You should see the mesh, costmap, and the robot moving towards the goal location as shown below:
<div align="center"><img src="../resources/isaac_sim_nvblox_nav2.gif"/></div>
