# Tutorial For Realsense-based Reconstruction

In this tutorial we'll demonstrate how one can perform depth-camera based reconstruction using a [realsense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html) camera, NVIDIA [VSLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam), and [nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox).

> Note: This tutorial has been tested with a realsense D455 or D435 connected to an x86 computer with an NVIDIA graphics card, as well as Jetson Xavier AGX.

## Installing the Dependencies  

Complete steps 1, 2, and 3 described in the [quickstart guide](../README.md#quickstart) to clone `isaac_ros_common` and `isaac_ros_nvblox` if you haven't done it already.

Then:

1. Download the code for the realsense ROS2 interface: 
   ```bash
   cd ~/workspaces/isaac_ros-dev/src
   ```
   ```bash
    git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2
   ```

> Note: Follow this [workaround](https://gitlab-master.nvidia.com/isaac_ros/isaac_ros_common/-/blob/release-dp1.1/docs/troubleshooting.md#realsense-driver-doesnt-work-with-ros2-humble) to build realsense2_camera package on ROS2 Humble.

2. Download the code for [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git).
   ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
   ```

3. Plug in your realsense camera (it needs to be plugged in before launching the container).

4. Launch the Docker container using the `run_dev.sh` script:
    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

5. At this point we can check that the realsense camera is connected by running `realsense-viewer`:
   
   ```bash
   realsense-viewer
   ```
   and turning on the "Stereo Module" in the GUI. You should see something like:

<div align="center"><img src="../resources/realsense_viewer.png" width=800px/></div>

6. If the `realsense-viewer` GUI prompts you to update the camera firmware, do so.

> Note: In our testing it was important that the camera had a recent version of the firmware installed.

7. Inside the container, install package-specific dependencies via `rosdep`:

    ```bash
    cd /workspaces/isaac_ros-dev/ && \
        rosdep install -i -r --from-paths src --rosdistro humble -y --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv nvblox"
    ```
    
8. Build and source the workspace:
    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

## Running the Example

To run the example, source the workspace 

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
```

And run the launchfile which launches the example

```bash
ros2 launch nvblox_examples_bringup nvblox_vslam_realsense.launch.py
```

Here is a few seconds of the result from running the example from my desk:

<div align="center"><img src="../resources/realsense_example.gif"/></div>
