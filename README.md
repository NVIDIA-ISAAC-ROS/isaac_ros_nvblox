# Isaac ROS Nvblox

Nvblox ROS 2 integration for local 3D scene reconstruction and mapping.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nvblox/isaac_sim_nvblox_humans.gif/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nvblox/isaac_sim_nvblox_humans.gif/" width="600px"/></a></div>

## Overview

[Isaac ROS Nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox) contains ROS 2 packages for 3D reconstruction and cost
maps for navigation. `isaac_ros_nvblox` processes depth and pose to
reconstruct a 3D scene in real-time and outputs a 2D costmap for
[Nav2](https://github.com/ros-planning/navigation2). The costmap is
used in planning during navigation as a vision-based solution to avoid
obstacles.

`isaac_ros_nvblox` is designed to work with depth-cameras and/or 3D LiDAR.
The package uses GPU acceleration to compute a 3D reconstruction and 2D costmaps using
[nvblox](https://github.com/nvidia-isaac/nvblox), the underlying
framework-independent C++ library.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox_nodegraph.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox_nodegraph.png/" width="750px"/></a></div>

Above is a typical graph that uses `isaac_ros_nvblox`.
Nvblox takes a depth image, a color image, and a pose as input, with
which it computes a 3D scene reconstruction on the GPU. In this graph
the pose is computed using `visual_slam`, or some other pose estimation
node. The reconstruction
is sliced into an output cost map which is provided through a cost map plugin
into [Nav2](https://github.com/ros-planning/navigation2).
An optional colorized 3D reconstruction is delivered into `rviz`
using the mesh visualization plugin. Nvblox streams mesh updates
to RViz to update the reconstruction in real-time as it is built.

`isaac_ros_nvblox` offers several modes of operation. In its default mode
the environment is assumed to be static. Two additional modes of operation are provided
to support mapping scenes which contain dynamic elements: human reconstruction, for
mapping scenes containing humans, and dynamic reconstruction, for mapping
scenes containing more general dynamic objects.
The graph above shows `isaac_ros_nvblox` operating in human reconstruction
mode. The color image corresponding to the depth image is processed with `unet`, using
the PeopleSemSegNet DNN model to estimate a segmentation mask for
persons in the color image. Nvblox uses this mask to separate reconstructed persons into a
separate humans-only part of the reconstruction. The [Technical Details](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/technical_details.html)
provide more information on these three types of mapping.

## Performance

The following tables provides timings for various functions of
[nvblox core](https://github.com/nvidia-isaac/nvblox) on various platforms.

<table class="docutils align-default">
    <thead>
        <tr class="row-odd">
            <th class="head">Dataset</th>
            <th class="head">Voxel Size (m)</th>
            <th class="head">Component</th>
            <th class="head">x86_64 w/ 4090 Ti (Desktop)</th>
            <th class="head">x86_64 w/ RTX3000 Ti (Laptop)</th>
            <th class="head">AGX Orin</th>
        </tr>
    </thead>
    <tbody>
        <tr class="row-even">
            <td rowspan="4">Replica</td>
            <td rowspan="4">0.05</td>
            <td>TSDF</td>
            <td>0.4 ms</td>
            <td>3.6 ms</td>
            <td>1.6 ms</td>
        </tr>
        <tr class="row-odd">
            <td>Color</td>
            <td>1.7 ms</td>
            <td>2.5 ms</td>
            <td>4.2 ms</td>
        </tr>
        <tr class="row-even">
            <td>Meshing</td>
            <td>1.6 ms</td>
            <td>4.0 ms</td>
            <td>12.3 ms</td>
        </tr>
        <tr class="row-odd">
            <td>ESDF</td>
            <td>1.9 ms</td>
            <td>8.4 ms</td>
            <td>8.4 ms</td>
        </tr>
        <tr class="row-even">
            <td rowspan="4">Redwood</td>
            <td rowspan="4">0.05</td>
            <td>TSDF</td>
            <td>0.2 ms</td>
            <td>0.2 ms</td>
            <td>0.5 ms</td>
        </tr>
        <tr class="row-odd">
            <td>Color</td>
            <td>1.1 ms</td>
            <td>1.6 ms</td>
            <td>2.4 ms</td>
        </tr>
        <tr class="row-even">
            <td>Meshing</td>
            <td>0.6 ms</td>
            <td>1.5 ms</td>
            <td>2.7 ms</td>
        </tr>
        <tr class="row-odd">
            <td>ESDF</td>
            <td>1.5 ms</td>
            <td>2.6 ms</td>
            <td>4.2 ms</td>
        </tr>
    </tbody>
</table>

---

## Documentation

Please visit the [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html) to learn how to use this repository.

---

## Packages

* [`isaac_ros_nvblox`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#quickstart)
  * [Try More Examples](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#try-more-examples)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#api)
    * [ROS Parameters](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/parameters.html)
    * [ROS Topics and Services](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/topics_and_services.html)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#troubleshooting)
    * [Isaac Sim Issues](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/troubleshooting/troubleshooting_nvblox_isaac_sim.html)
    * [RealSense Issues](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/troubleshooting/troubleshooting_nvblox_realsense.html)
    * [ROS Communication Issues](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/troubleshooting/troubleshooting_nvblox_ros_communication.html)
* [`nvblox_examples_bringup`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/nvblox_examples_bringup/index.html)
* [`nvblox_image_padding`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/nvblox_image_padding/index.html)
* [`nvblox_isaac_sim`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/nvblox_isaac_sim/index.html)
* [`nvblox_msgs`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/nvblox_msgs/index.html)
* [`nvblox_nav2`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/nvblox_nav2/index.html)
* [`nvblox_performance_measurement`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/nvblox_performance_measurement/index.html)
* [`nvblox_ros`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/nvblox_ros/index.html)
* [`nvblox_ros_common`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/nvblox_ros_common/index.html)
* [`nvblox_rviz_plugin`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/nvblox_rviz_plugin/index.html)
* [`realsense_splitter`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/realsense_splitter/index.html)
* [`semantic_label_conversion`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/semantic_label_conversion/index.html)

## Latest

Update 2023-10-18: General dynamic reconstruction.
