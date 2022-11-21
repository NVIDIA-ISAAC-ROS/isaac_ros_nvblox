# Isaac ROS Nvblox (Preview)

<div align="center"><img src="resources/nvblox_navigation_trim.gif" width=800px/></div>

## Overview

Nvblox is a package for building a 3D reconstruction of the environment around your robot from sensor observations in real-time. The reconstruction is intended to be used by path planners to generate collision-free paths. Under the hood, nvblox uses NVIDIA CUDA to accelerate this task to allow operation at real-time rates. This repository contains ROS2 integration for the [nvblox core library](https://github.com/nvidia-isaac/nvblox).

Given a stream of depth images and the corresponding pose of the depth sensor, Isaac ROS Nvblox produces both a 2D distance map slice, representing the distance from each point to the nearest reconstructed object, and also a 3D mesh for visualization in RVIZ. An optional RGB stream can also be used to color the reconstruction for visualization.

Read more about Nvblox's technical details [here](docs/technical-details.md).

The figure below shows a simple system utilizing nvblox for path planning.

<div align="center"><img src="resources/system_diagram.png" width=800px/></div>

## Table of Contents

- [Isaac ROS Nvblox (Preview)](#isaac-ros-nvblox-preview)
  - [Overview](#overview)
  - [Table of Contents](#table-of-contents)
  - [Latest Update](#latest-update)
  - [Supported Platforms](#supported-platforms)
    - [Docker](#docker)
  - [Quickstart](#quickstart)
  - [Next Steps](#next-steps)
    - [Try More Examples](#try-more-examples)
    - [Customize your Dev Environment](#customize-your-dev-environment)
  - [Package Reference](#package-reference)
    - [`nvblox_nav2`](#nvblox_nav2)
      - [Usage](#usage)
      - [ROS Parameters](#ros-parameters)
      - [ROS Topics Subscribed](#ros-topics-subscribed)
      - [ROS Topics Published](#ros-topics-published)
      - [ROS Services Advertised](#ros-services-advertised)
  - [Troubleshooting](#troubleshooting)
    - [Isaac ROS Troubleshooting](#isaac-ros-troubleshooting)
    - [`realsense-ros` packages don't build with ROS2 Humble](#realsense-ros-packages-dont-build-with-ros2-humble)
  - [Updates](#updates)

## Latest Update

Update 2022-10-19: Updated OSS licensing

## Supported Platforms

This package is designed and tested to be compatible with ROS2 Humble running on [Jetson](https://developer.nvidia.com/embedded-computing) or an x86_64 system with an NVIDIA GPU.

> **Note**: Versions of ROS2 earlier than Humble are **not** supported. This package depends on specific ROS2 implementation features that were only introduced beginning with the Humble release.

| Platform | Hardware                                                                                                                                                                                                 | Software                                                                                                             | Notes                                                                                                                                                                                   |
| -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Jetson   | [Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) <br> [Jetson Xavier](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-agx-xavier/) | [JetPack 5.0.2](https://developer.nvidia.com/embedded/jetpack)                                                       | For best performance, ensure that [power settings](https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance.html) are configured appropriately. |
| x86_64   | NVIDIA GPU                                                                                                                                                                                               | [Ubuntu 20.04+](https://releases.ubuntu.com/20.04/) <br> [CUDA 11.6.1+](https://developer.nvidia.com/cuda-downloads) |

### Docker

To simplify development, we strongly recommend leveraging the Isaac ROS Dev Docker images by following [these steps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md). This will streamline your development environment setup with the correct versions of dependencies on both Jetson and x86_64 platforms.

> **Note:** All Isaac ROS Quickstarts, tutorials, and examples have been designed with the Isaac ROS Docker images as a prerequisite.

## Quickstart

1. Set up your development environment by following the instructions [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md).

2. Complete the RealSense setup guide [here](https://github.com/NVIDIA-ISAAC-ROS/.github/blob/main/profile/realsense-setup.md).

3. Clone this repository and its dependencies under `~/workspaces/isaac_ros-dev/src`.

    ```bash
    cd ~/workspaces/isaac_ros-dev/src
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
    ```

    ```bash
    git clone --recurse-submodules https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox && \
        cd isaac_ros_nvblox && git lfs pull
    ```

4. Configure the container created by `isaac_ros_common/scripts/run_dev.sh` to include `librealsense`. Create the `.isaac_ros_common-config` file in the `isaac_ros_common/scripts` directory:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts && \
    touch .isaac_ros_common-config && \
    echo CONFIG_IMAGE_KEY=humble.nav2.realsense > .isaac_ros_common-config
    ```

5. Pull down a ROS Bag of sample data:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_nvblox && \ 
      git lfs pull -X "" -I "nvblox_ros/test/test_cases/rosbags/nvblox_pol"
    ```

6. Launch the Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

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

9. (Optional) Run tests to verify complete and correct installation:  

    ```bash
    colcon test --executor sequential
    ```

10. In a **current terminal** inside the Docker container, run the launch file for Nvblox with `nav2`:

    ```bash
    source /workspaces/isaac_ros-dev/install/setup.bash && \
        ros2 launch nvblox_nav2 carter_sim.launch.py
    ```

11. Open a **second terminal** inside the docker container:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

12. In the **second terminal**, play the ROS Bag:

    ```bash
    ros2 bag play src/isaac_ros_nvblox/nvblox_ros/test/test_cases/rosbags/nvblox_pol
    ```

You should see the robot reconstructing a mesh, with a costmap overlaid on top.

## Next Steps

### Try More Examples

To continue your exploration, check out the following suggested examples:

- [Tutorial with Isaac Sim](./docs/tutorial-isaac-sim.md)
- [Tutorial with realsense, VSLAM and nvblox](./docs/tutorial-nvblox-vslam-realsense.md)

### Customize your Dev Environment

To customize your development environment, reference [this guide](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/modify-dockerfile.md).

## Package Reference

### `nvblox_nav2`

#### Usage

```bash
ros2 launch nvblox_nav2 carter_sim.launch.py
```

#### ROS Parameters

| ROS Parameter                                 | Type     | Default | Description                                                                                                                                                                         |
| --------------------------------------------- | -------- | ------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `voxel_size`                                  | `float`  | `0.05`  | Voxel size (in meters) to use for the map.                                                                                                                                          |
| `esdf`                                        | `bool`   | `true`  | Whether to compute the ESDF (map of distances to the nearest obstacle).                                                                                                             |
| `esdf_2d`                                     | `bool`   | `true`  | Whether to compute the ESDF in 2D (true) or 3D (false).                                                                                                                             |
| `distance_slice`                              | `bool`   | `true`  | Whether to output a distance slice of the ESDF to be used for path planning.                                                                                                        |
| `mesh`                                        | `bool`   | `true`  | Whether to output a mesh for visualization in rviz, to be used with `nvblox_rviz_plugin`.                                                                                           |
| `global_frame`                                | `string` | `map`   | The name of the TF frame to be used as the global frame.                                                                                                                            |
| `slice_height`                                | `float`  | `1.0`   | The *output* slice height for the distance slice and ESDF pointcloud. Does not need to be within min and max height below. In units of meters.                                      |
| `min_height`                                  | `float`  | `0.0`   | The minimum height, in meters, to consider obstacles part of the 2D ESDF slice.                                                                                                     |
| `max_height`                                  | `float`  | `1.0`   | The maximum height, in meters, to consider obstacles part of the 2D ESDF slice.                                                                                                     |
| `max_tsdf_update_hz`                          | `float`  | `10.0`  | The maximum rate (in Hz) at which to integrate depth images into the TSDF. A value of 0.0 means there is no cap.                                                                    |
| `max_color_update_hz`                         | `float`  | `5.0`   | The maximum rate (in Hz) at which to integrate color images into the color layer. A value of 0.0 means there is no cap.                                                             |
| `max_mesh_update_hz`                          | `float`  | `5.0`   | The maximum rate (in Hz) at which to update and color the mesh. A value of 0.0 means there is no cap.                                                                               |
| `max_esdf_update_hz`                          | `float`  | `2.0`   | The maximum rate (in Hz) at which to update the ESDF and output the distance slice. A value of 0.0 means there is no cap.                                                           |
| `tsdf_integrator_max_integration_distance_m`  | `float`  | `10.0`  | The maximum distance, in meters, to integrate the TSDF up to.                                                                                                                       |
| `tsdf_integrator_truncation_distance_vox`     | `float`  | `4.0`   | The truncation distance, in units of voxels, for the TSDF.                                                                                                                          |
| `tsdf_integrator_max_weight`                  | `float`  | `100.0` | Maximum weight for the TSDF. Setting this number higher will lead to higher-quality reconstructions but worse performance in dynamic scenes.                                        |
| `mesh_integrator_min_weight`                  | `float`  | `1e-4`  | Minimum weight of the TSDF to consider for inclusion in the mesh.                                                                                                                   |
| `mesh_integrator_weld_vertices`               | `bool`   | `false` | Whether to weld identical vertices together in the mesh. Currently reduces the number of vertices by a factor of 5x, but is quite slow so we do not recommend you use this setting. |
| `color_integrator_max_integration_distance_m` | `float`  | `10.0`  | Maximum distance, in meters, to integrate the color up to.                                                                                                                          |
| `esdf_integrator_min_weight`                  | `float`  | `1e-4`  | Minimum weight of the TSDF to consider for inclusion in the ESDF.                                                                                                                   |
| `esdf_integrator_min_site_distance_vox`       | `float`  | `1.0`   | Minimum distance to consider a voxel within a surface for the ESDF calculation.                                                                                                     |
| `esdf_integrator_max_distance_m`              | `float`  | `10.0`  | Maximum distance to compute the ESDF up to, in meters.                                                                                                                              |

#### ROS Topics Subscribed

| ROS Topic           | Interface                                                                                                      | Description                                                                                                                                                                                   |
| ------------------- | -------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `depth/image`       | [sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg)           | The input depth image to be integrated. Must be paired with a `camera_info` message below. Supports both floating-point (depth in meters) and `uint16` (depth in millimeters, OpenNI format). |
| `depth/camera_info` | [sensor_msgs/CameraInfo](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg) | Required topic along with the depth image; contains intrinsic calibration parameters of the depth camera.                                                                                     |
| `color/image`       | [sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)           | Optional input color image to be integrated. Must be paired with a `camera_info` message below. Only used to color the mesh.                                                                  |
| `color/camera_info` | [sensor_msgs/CameraInfo](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg) | Optional topic along with the color image above, contains intrinsics of the color camera.                                                                                                     |

#### ROS Topics Published

| ROS Topic    | Interface                                                                                                                           | Description                                                                                                                                                                          |
| ------------ | ----------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `mesh`       | [nvblox_msgs/Mesh](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/blob/main/nvblox_msgs/msg/Mesh.msg)                         | A visualization topic showing the mesh produced from the TSDF in a form that can be seen in RViz using `nvblox_rviz_plugin`. Set ``max_mesh_update_hz`` to control its update rate.  |
| `pointcloud` | [sensor_msgs/PointCloud2](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/PointCloud2.msg)                    | A pointcloud of the 2D ESDF (Euclidean Signed Distance Field), with intensity as the metric distance to the nearest obstacle. Set ``max_esdf_update_hz`` to control its update rate. |
| `map_slice`  | [nvblox_msgs/DistanceMapSlice](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/blob/main/nvblox_msgs/msg/DistanceMapSlice.msg) | A 2D slice of the ESDF, to be consumed by `nvblox_nav2` package for interfacing with Nav2. Set ``max_esdf_update_hz`` to control its update rate.                                    |

#### ROS Services Advertised

| ROS Service | Interface                                                                                      | Description                                                                                                                                         |
| ----------- | ---------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| `save_ply`  | [std_srvs/Empty](https://github.com/ros2/common_interfaces/blob/master/std_srvs/srv/Empty.srv) | This service has an empty request and response. Calling this service will write a mesh to disk at the path specified by the `output_dir` parameter. |

## Troubleshooting

### Isaac ROS Troubleshooting

For solutions to problems with Isaac ROS, please check [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/troubleshooting.md).

### `realsense-ros` packages don't build with ROS2 Humble

Please follow the workaround [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/troubleshooting.md#realsense-driver-doesnt-work-with-ros2-humble).

## Updates

| Date       | Changes                                                                                                                                   |
| ---------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| 2022-10-19 | Updated OSS licensing                                                                                                                     |
| 2022-08-31 | Update to be compatible with JetPack 5.0.2. Serialization of nvblox maps to file. Support for 3D LIDAR input and performace improvements. |
| 2022-06-30 | Support for ROS2 Humble and miscellaneous bug fixes.                                                                                      |
| 2022-03-21 | Initial version.                                                                                                                          |
