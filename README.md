# Isaac ROS Nvblox

Nvblox ROS 2 integration for local 3D scene reconstruction and mapping.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-4.0/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nvblox/isaac_sim_nvblox_humans.gif/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-4.0/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nvblox/isaac_sim_nvblox_humans.gif/" width="600px"/></a></div>

## Overview

Isaac ROS Nvblox contains ROS 2 packages for 3D reconstruction and cost
maps for navigation. `isaac_ros_nvblox` processes depth and pose to
reconstruct a 3D scene in real-time and outputs a 2D costmap for
[Nav2](https://github.com/ros-navigation/navigation2). The costmap is
used in planning during navigation as a vision-based solution to avoid
obstacles.

`isaac_ros_nvblox` is designed to work with depth-cameras and/or 3D LiDAR.
The package uses GPU acceleration to compute a 3D reconstruction and 2D costmaps using
[nvblox](https://github.com/nvidia-isaac/nvblox), the underlying
framework-independent C++ library.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-4.0/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox_nodegraph.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-4.0/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox_nodegraph.png/" width="750px"/></a></div>

Above is a typical graph that uses `isaac_ros_nvblox`.
Nvblox takes a depth image, a color image, and a pose as input, with
which it computes a 3D scene reconstruction on the GPU. In this graph
the pose is computed using `visual_slam`, or some other pose estimation
node. The reconstruction
is sliced into an output cost map which is provided through a cost map plugin
into [Nav2](https://github.com/ros-navigation/navigation2).
An optional colorized 3D reconstruction is delivered into `rviz`
using the mesh visualization plugin. Nvblox streams mesh updates
to RViz to update the reconstruction in real-time as it is built.

`isaac_ros_nvblox` offers several modes of operation. In its default mode
the environment is assumed to be static. Two additional modes of operation are provided
to support mapping scenes which contain dynamic elements: people reconstruction, for
mapping scenes containing people, and dynamic reconstruction, for mapping
scenes containing more general dynamic objects.
The graph above shows `isaac_ros_nvblox` operating in people reconstruction
mode. The color image corresponding to the depth image is processed with `unet`, using
the PeopleSemSegNet DNN model to estimate a segmentation mask for
persons in the color image. Nvblox uses this mask to separate reconstructed persons into a
separate people-only part of the reconstruction. The [Technical Details](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/technical_details.html)
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
            <th class="head">x86_64 + 5090 (Desktop)</th>
            <th class="head">x86_64 + 3090 (Desktop)</th>
            <th class="head">x86_64 + A3000 (Laptop)</th>
            <th class="head">AGX Thor</th>
            <th class="head">AGX Orin</th>
            <th class="head">Orin Nano</th>
        </tr>
    </thead>
    <tbody>
        <tr class="row-even">
            <td rowspan="5">Replica</td>
            <td rowspan="5">0.05</td>
            <td>TSDF</td> <!-- tsdf/integrate -->
            <td>0.1 ms</td>
            <td>0.5 ms</td>
            <td>0.3 ms</td>
            <td>0.4 ms</td>
            <td>0.8 ms</td>
            <td>2.1 ms</td>
        </tr>
        <tr class="row-odd">
            <td>Color</td> <!-- color/integrate -->
            <td>0.3 ms</td>
            <td>0.7 ms</td>
            <td>0.7 ms</td>
            <td>0.8 ms</td>
            <td>1.1 ms</td>
            <td>3.6 ms</td>
        </tr>
        <tr class="row-even">
            <td>Meshing</td> <!-- mesh/integrate -->
            <td>0.3 ms</td>
            <td>0.7 ms</td>
            <td>1.3 ms</td>
            <td>1.4 ms</td>
            <td>2.3 ms</td>
            <td>13 ms</td>
        </tr>
        <tr class="row-odd">
            <td>ESDF</td> <!-- esdf/integrate -->
            <td>0.3 ms</td>
            <td>0.8 ms</td>
            <td>1.2 ms</td>
            <td>1.0 ms</td>
            <td>1.7 ms</td>
            <td>6.2 ms</td>
        </tr>
        <tr class="row-even">
            <td>Dynamics</td> <!-- sum(multi_mapper/integrate_depth/dynamic_block) -->
            <td>0.7 ms</td>
            <td>1.7 ms</td>
            <td>1.4 ms</td>
            <td>1.4 ms</td>
            <td>2.0 ms</td>
            <td>N/A(\*)</td>
        </tr>
        <tr class="row-even">
            <td rowspan="5">Redwood</td>
            <td rowspan="5">0.05</td>
            <td>TSDF</td>
            <td>0.09 ms</td>
            <td>0.2 ms</td>
            <td>0.2 ms</td>
            <td>0.2 ms</td>
            <td>0.5 ms</td>
            <td>1.2 ms</td>
        </tr>
        <tr class="row-odd">
            <td>Color</td>
            <td>0.3 ms</td>
            <td>0.5 ms</td>
            <td>0.5 ms</td>
            <td>0.6 ms</td>
            <td>0.8 ms</td>
            <td>2.6 ms</td>
        </tr>
        <tr class="row-even">
            <td>Meshing</td>
            <td>0.2 ms</td>
            <td>0.3 ms</td>
            <td>0.5 ms</td>
            <td>0.8 ms</td>
            <td>0.9 ms</td>
            <td>4.2 ms</td>
        </tr>
        <tr class="row-odd">
            <td>ESDF</td>
            <td>0.3 ms</td>
            <td>0.8 ms</td>
            <td>1.0 ms</td>
            <td>1.0 ms</td>
            <td>1.5 ms</td>
            <td>5.1 ms</td>
        </tr>
        <tr class="row-even">
            <td>Dynamics</td>
            <td>0.4 ms</td>
            <td>1.0 ms</td>
            <td>0.7 ms</td>
            <td>0.8 ms</td>
            <td>1.2 ms</td>
            <td>N/A(\*)</td>
        </tr>
     </tbody>
</table>

(\*): Dynamics not supported on Jetson Nano.

---

## Documentation

Please visit the [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html) to learn how to use this repository.

---

## Packages

* [`isaac_ros_nvblox`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#quickstart)
    * [Set Up Development Environment](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#set-up-development-environment)
    * [Download Quickstart Assets](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#download-quickstart-assets)
    * [Set Up `isaac_ros_nvblox`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#set-up-package-name)
    * [Run Example Launch File](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#run-example-launch-file)
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
* [`nvblox_msgs`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/nvblox_msgs/index.html)
* [`nvblox_nav2`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/nvblox_nav2/index.html)
* [`nvblox_ros`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/nvblox_ros/index.html)
* [`nvblox_ros_common`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/nvblox_ros_common/index.html)
* [`nvblox_rviz_plugin`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/nvblox_rviz_plugin/index.html)
* [`realsense_splitter`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/realsense_splitter/index.html)
* [`semantic_label_conversion`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/semantic_label_conversion/index.html)

## Latest

Update 2025-10-24: Support for ROS 2 Jazzy
