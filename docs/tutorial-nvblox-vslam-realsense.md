# Tutorial For RealSense-based Reconstruction

This tutorial demonstrates how to perform depth-camera based reconstruction using a [Realsense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html) camera, NVIDIA [VSLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam), and [nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox).

> Note: This tutorial has been tested with a Realsense D455/D435 camera connected to an x86 computer with an NVIDIA graphics card, as well as a Jetson Xavier AGX.

## Realsense Camera Firmware

This example is tested and compatible with realsense camera firmware version 5.13.0.50 which is available [here](https://dev.intelrealsense.com/docs/firmware-releases).

> Note: We experienced issues with the latest realsense firmware (version 5.14 at the time of publishing). It's possible at some point that this starts working, but our recommendation is to install *exactly* 5.13.0.50.

## Host System Setup

We have found ROS2 message delivery to be unreliable under high load without some small modifications to the QoS profile (especially on weaker machines). Before running this example run
```
sudo sysctl -w net.core.rmem_max=8388608 net.core.rmem_default=8388608
```
However this only sets these parameters until reboot. To set them permanently run:
```
echo -e "net.core.rmem_max=8388608\nnet.core.rmem_default=8388608\n" | sudo tee /etc/sysctl.d/60-cyclonedds.conf
```
More details on DDS tuning can be found [here](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html).

## Installing the Dependencies  

1. Complete steps 1, 2, and 3 described in the [quickstart guide](../README.md#quickstart) to clone `isaac_ros_common` and
   `isaac_ros_nvblox` and to build [librealsense](https://github.com/IntelRealSense/librealsense) as part of the Docker container.

2. Complete the [RealSense setup tutorial](https://github.com/NVIDIA-ISAAC-ROS/.github/blob/main/profile/realsense-setup.md).

3. Download the code for [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git).

   ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
   ```

4. Launch the Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

5. Inside the container, install package-specific dependencies via `rosdep`:

    ```bash
    cd /workspaces/isaac_ros-dev/ && \
        rosdep install -i -r --from-paths src --rosdistro humble -y --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv nvblox"
    ```

6. Build and source the workspace:

    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

## Running the Example

To run the example, source the workspace:

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
```

Then run the launch file, which launches the example:

```bash
ros2 launch nvblox_examples_bringup nvblox_vslam_realsense.launch.py
```

Here is a few seconds of the result from running the example:

<div align="center"><img src="../resources/realsense_example.gif"/></div>

## Troubleshooting

See our troubleshooting page [here](troubleshooting-nvblox-vslam-realsense.md).
