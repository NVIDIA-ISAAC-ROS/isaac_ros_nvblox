# Isaac ROS Nvblox

<div align="center"><img src="resources/isaac_sim_nvblox_humans.gif" width=400px/></div>

## Overview

Isaac ROS Nvblox contains ROS 2 packages for 3D reconstruction and cost maps for navigation. `nvblox_nav2` processes depth and pose to reconstruct a 3D scene in real-time and a 2D costmap for [Nav2](https://github.com/ros-planning/navigation2). The costmap is used in planning during navigation as a vision-based solution to avoid obstacles.  

`nvblox_nav2` is designed to work with stereo cameras, which provide a depth image, and the corresponding pose uses GPU acceleration to compute 3D reconstruction and 2D costmaps using [nvblox](https://github.com/nvidia-isaac/nvblox).

<div align="center"><img src="resources/isaac_ros_nvblox_nodegraph.png" width=700px/></div>

Above is a typical graph that uses `nvblox_nav2`. The input color image corresponding to the depth image is processed with `unet`, using the PeopleSemSegNet DNN model to estimate a segmentation mask for persons in the color image. Pose corresponding to the depth image is computed using `visual_slam`. The resulting persons mask and pose is used with the color image and depth image to perform 3D scene reconstruction. The output costmap is provided through a cost map plugin into [Nav2](https://github.com/ros-planning/navigation2), with an optional colorized 3D reconstruction into a Rviz using the mesh visualization plugin.

<div align="center"><img src="resources/isaac_ros_nvblox_person_removal.png" width=750px/></div>

`nvblox_nav2` builds the reconstructed map in the form of a TSDF (Truncated Signed Distance Function) stored in a 3D voxel grid. This approach is similar to 3D occupancy grid mapping approaches in which occupancy probabilities are stored at each voxel. However, TSDF-based approaches like nvblox store the (signed) distance to the closest surface at each voxel. The surface of the environment can then be extracted as the zero-level set of this voxelized function. Typically, TSDF-based reconstructions provide higher quality surface reconstructions.

In addition to their use in reconstruction, distance fields are also useful for path planning because they provide an immediate means of checking whether potential future robot positions are in collision. 

People are common obstacles for mobile robots, and while part of a costmap, people should not be part of the 3D reconstruction.  Planners that provide behavioral awareness by navigating differently depending on their proximity to people, benefit from a costmap for people. Person segmentation is computed using the color image, with the resulting mask applied to the depth image separating depth into scene depth and person depth images. The scene depth image is forwarded to TSDF mapping as explained above, the depth image for people is processed to an occupancy grid map.

To relax the assumption that occupancy grid maps only capture static objects, Nvblox applies an occupancy decay step. At a fixed frequency, all voxel occupancy probabilities are decayed towards 0.5 over time. This means that the state of the map (occupied or free) becomes less certain after it has fallen out of the field of view, until it becomes unknown (0.5 occupancy probability).


## Table of Contents

- [Isaac ROS Nvblox](#isaac-ros-nvblox)
  - [Overview](#overview)
  - [Table of Contents](#table-of-contents)
  - [Latest Update](#latest-update)
  - [Supported Platforms](#supported-platforms)
    - [Docker](#docker)
  - [Quickstart](#quickstart)
  - [Next Steps](#next-steps)
    - [Try More Examples](#try-more-examples)
    - [Customize your Dev Environment](#customize-your-dev-environment)
  - [Packages Overview](#packages-overview)
  - [ROS 2 Parameters](#ros-2-parameters)
  - [ROS 2 Topics and Services](#ros-2-topics-and-services)
  - [Troubleshooting](#troubleshooting)
    - [Isaac ROS Troubleshooting](#isaac-ros-troubleshooting)
    - [*realsense-ros* packages don't build with ROS 2 Humble](#realsense-ros-packages-dont-build-with-ros-2-humble)
    - [Troubleshooting the Nvblox Realsense Example](#troubleshooting-the-nvblox-realsense-example)
    - [Troubleshooting ROS 2 communication issues](#troubleshooting-ros-2-communication-issues)
  - [Updates](#updates)

## Latest Update

Update 2023-04-05: Human reconstruction and new weighting functions.

## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on [Jetson](https://developer.nvidia.com/embedded-computing) or an x86_64 system with an NVIDIA GPU.

> **Note**: Versions of ROS 2 earlier than Humble are **not** supported. This package depends on specific ROS 2 implementation features that were only introduced beginning with the Humble release.

| Platform | Hardware                                                                                                                                                                                                 | Software                                                                                                           | Notes                                                                                                                                                                                   |
| -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Jetson   | [Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) <br> [Jetson Xavier](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-agx-xavier/) | [JetPack 5.1.1](https://developer.nvidia.com/embedded/jetpack)                                                     | For best performance, ensure that [power settings](https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance.html) are configured appropriately. |
| x86_64   | NVIDIA GPU                                                                                                                                                                                               | [Ubuntu 20.04+](https://releases.ubuntu.com/20.04/) <br> [CUDA 11.8+](https://developer.nvidia.com/cuda-downloads) |                                                                                                                                                                                         |

### Docker

To simplify development, we strongly recommend leveraging the Isaac ROS Dev Docker images by following [these steps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md). This will streamline your development environment setup with the correct versions of dependencies on both Jetson and x86_64 platforms.

> **Note**: All Isaac ROS Quickstarts, tutorials, and examples have been designed with the Isaac ROS Docker images as a prerequisite.

## Quickstart

1. Set up your development environment by following the instructions [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md).

2. Clone this repository and its dependencies under `~/workspaces/isaac_ros-dev/src` or `/ssd/workspaces/isaac_ros-dev/src` depending upon SD card or SSD setup.

    ```bash
    cd ~/workspaces/isaac_ros-dev/src
    ```
    > **Note**: For Jetson setup with SSD as optional storage:
    >  ```bash
    >  cd /ssd/workspaces/isaac_ros-dev/src
    >  ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
    ```

    ```bash
    git clone --recurse-submodules https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox && \
        cd isaac_ros_nvblox && git lfs pull
    ```

3. Pull down a ROS Bag of sample data:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_nvblox && \ 
      git lfs pull -X "" -I "nvblox_ros/test/test_cases/rosbags/nvblox_pol"
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

8. (Optional) Run tests to verify complete and correct installation:  

    ```bash
    colcon test --executor sequential
    ```

9. In a **current terminal** inside the Docker container, run the launch file for Nvblox with `nav2`:

    ```bash
    source /workspaces/isaac_ros-dev/install/setup.bash && \
        ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py
    ```

10. Open a **second terminal** inside the docker container:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

11. In the **second terminal**, play the ROS Bag:

    ```bash
    ros2 bag play src/isaac_ros_nvblox/nvblox_ros/test/test_cases/rosbags/nvblox_pol
    ```

You should see the robot reconstructing a mesh, with the 2d esdf slice overlaid on top.

<div align="center"><img src="resources/basic_example_rviz.png" width=500px/></div>

## Next Steps

### Try More Examples

The launch files for all examples are available in the `nvblox_examples_bringup` package:
| Launch file                          | Arguments                           | Description                                                                                                                           |
| ------------------------------------ | ----------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| `isaac_sim_example.launch.py`        | `run_nav2`, `run_rviz`              | Example to run with Isaac Sim ([tutorial](./docs/tutorial-isaac-sim.md))                                                              |
| `isaac_sim_humans_example.launch.py` | `run_nav2`, `run_rviz`              | Example to run with Isaac Sim including human reconstruction ([tutorial](./docs/tutorial-human-reconstruction-isaac-sim.md))          |
| `realsense_example.launch.py`        | `from_bag`, `bag_path`,  `run_rviz` | Example to run with a realsense camera ([tutorial](./docs/tutorial-realsense.md))                                                     |
| `realsense_humans_example.launch.py` | `from_bag`, `bag_path`,  `run_rviz` | Example to run with a realsense camera including human reconstruction ([tutorial](./docs/tutorial-human-reconstruction-realsense.md)) |
| `record_realsense.launch.py`         | `launch_realsense`, `run_rqt`       | Record realsense data to replay with the above examples ([tutorial](./docs/tutorial-realsense-record.md))                             |


### Customize your Dev Environment

To customize your development environment, reference [this guide](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/modify-dockerfile.md).

## Packages Overview

* **isaac_ros_nvblox**: A meta-package.
* **nvblox_examples_bringup**: Launch files and configurations for launching the examples.
* **nvblox_image_padding**: Node to pad and crop images for adjusting the image size to the fixed input resolution that is required by the [image segmentation network](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_segmentation).
* **nvblox_isaac_sim**: Contains scripts for launching Isaac Sim configured for use with nvblox.
* **realsense_splitter**: Node for using the realsense camera with inbuilt projector. See why this is needed [here](./docs/troubleshooting-nvblox-realsense.md).
* **semantic_label_conversion**: Package for converting semantic labels coming from Isaac Sim to mask images used by nvblox ([readme](./nvblox_examples/semantic_label_conversion/README.md)).
* **nvblox_msgs**: Custom messages for transmitting the output distance map slice and mesh over ROS 2.
* **nvblox_nav2**: Contains a custom plugin that allows ROS 2 Nav2 to consume nvblox distance map outputs.
* **nvblox_performance_measurement**: Multiple packages containing tools for measuring nvblox performance ([readme](./nvblox_performance_measurement/README.md)).
* **nvblox_ros**: The ROS 2 wrapper for the core reconstruction library and the nvblox node.
* **nvblox_ros_common**: Package providing repository wide utility functions.
* **nvblox_rviz_plugin**: A plugin for displaying nvblox's (custom) mesh type in RVIZ.
* **\[submodule\] nvblox**: The core (ROS independent) reconstruction library.


## ROS 2 Parameters

Find all available ROS 2 parameters [here](./docs/parameters.md).

## ROS 2 Topics and Services

Find all ROS 2 subscribers, publishers and services [here](./docs/topics-and-services.md).

## Troubleshooting

### Isaac ROS Troubleshooting

For solutions to problems with Isaac ROS, please check [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/troubleshooting.md).

### *realsense-ros* packages don't build with ROS 2 Humble

Please follow the workaround [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/troubleshooting.md#realsense-driver-doesnt-work-with-ros2-humble).

### Troubleshooting the Nvblox Realsense Example

See our troubleshooting page [here](./docs/troubleshooting-nvblox-realsense.md).

### Troubleshooting ROS 2 communication issues

If it looks like you are dropping messages or you are not receiving any messages, please consult our troubleshooting page [here](./docs/troubleshooting-nvblox-ros2.md).

## Updates

| Date       | Changes                                                                                                                                    |
| ---------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| 2023-04-05 | Human reconstruction and new weighting functions.                                                                                          |
| 2022-12-10 | Updated documentation.                                                                                                                     |
| 2022-10-19 | Updated OSS licensing                                                                                                                      |
| 2022-08-31 | Update to be compatible with JetPack 5.0.2. Serialization of nvblox maps to file. Support for 3D LIDAR input and performance improvements. |
| 2022-06-30 | Support for ROS 2 Humble and miscellaneous bug fixes.                                                                                      |
| 2022-03-21 | Initial version.                                                                                                                           |
