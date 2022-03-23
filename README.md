# Isaac ROS Nvblox (Preview)

Nvblox is a package for building a 3D reconstruction of the environment around your robot from sensor observations in real-time. The reconstruction is intended to be used by path planners to generate collision-free paths. Under the hood, nvblox uses NVIDIA CUDA to accelerate this task to allow operation at real-time rates. This repository contains ROS2 integration for the [nvblox core library](https://github.com/nvidia-isaac/nvblox).

Below is an example of the nvblox being used with ROS2 Nav2 for real-time reconstruction and navigation in Isaac Sim.
<div align="center"><img src="docs/images/nvblox_navigation_trim.gif" width=800px/></div>

For solutions to known issues, please visit the [Troubleshooting](#troubleshooting) section below.

# Input/Outputs

Here we discuss the inputs you have to provide to nvblox, and the outputs it produces for downstream tasks.

Inputs:
* **Depth Images**: We require input from a sensor supplying depth per pixel. Examples of such sensors are the Intel Realsense series and Kinect cameras.
* **Sensor Pose**: We require localization of the depth sensor as input to nvblox
* [Optional] **Color Images**: These can be used to color the reconstruction for visualization.

Outputs:
* **Distance map slice**: A 2D map that expresses the distance at each point from objects reconstructed in the environment. This is typically used by a planner to compute a collision cost map.
* **Mesh**: We output a mesh for visualization in RVIZ.

The figure below shows a simple system utilizing nvblox for path planning.

<div align="center"><img src="docs/images/system_diagram.png" width=800px/></div>

# Packages in this repository

* **isaac_ros_nvblox**: A meta-package.
* **nvblox_isaac_sim**: Contains scripts for launching Isaac Sim configured for use with nvblox.
* **nvblox_msgs**: Custom messages for transmitting the output distance map slice and mesh over ROS2.
* **nvblox_nav2**: Contains a custom plugin that allows ROS2 Nav2 to consume nvblox distance map outputs, as well as launch files for launching a navigation solution for use in simulation.
* **nvblox_ros**: The ROS2 wrapper for the core reconstruction library and the nvblox node.
* **nvblox_rviz_plugin**: A plugin for displaying nvblox's (custom) mesh type in RVIZ.
* **[submodule] nvblox**: The core (ROS independent) reconstruction library.

# (Brief) Technical Details

Nvblox builds the reconstructed map in the form of a Truncated Signed Distance Function (TSDF) stored in a 3D voxel grid. This approach is similar to 3D occupancy grid mapping approaches in which occupancy probabilities are stored at each voxel. In contrast however, TSDF-based approaches (like nvblox) store the (signed) distance to the closest surface at each voxel. The surface of the environment can then be extracted as the zero-level set of this voxelized function. Typically TSDF-based reconstructions provide higher quality surface reconstructions.

In addition to their use in reconstruction, distance fields are also useful for path planning because they provide an immediate means of checking whether potential future robot positions are in collision. This fact, the utility of distance functions for both reconstruction and planning, motivates their use in nvblox (a reconstruction library for path planning).


# System Requirements
This Isaac ROS package is designed and tested to be compatible with ROS2 Foxy on Jetson hardware.

## Jetson
- [Jetson AGX Xavier or Xavier NX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/)
- [JetPack 4.6.1](https://developer.nvidia.com/embedded/jetpack)

## x86_64
- Ubuntu 20.04+
- CUDA 11.4+ supported discrete GPU with 2+ GB of VRAM

**Note:** If running [Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/requirements.html), more VRAM would be required.

**Note:** For best performance on Jetson, ensure that power settings are configured appropriately ([Power Management for Jetson](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/power_management_jetson_xavier.html#wwpID0EUHA)).

### Docker
Precompiled ROS2 Foxy packages are not available for JetPack 4.6.1 (based on Ubuntu 18.04 Bionic). You can either manually compile ROS2 Foxy and required dependent packages from source or use the Isaac ROS development Docker image from [Isaac ROS Common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common).

You must first install the [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) to make use of the Docker container development/runtime environment.

Configure `nvidia-container-runtime` as the default runtime for Docker by editing `/etc/docker/daemon.json` to include the following:
```
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
```
Then restart Docker: `sudo systemctl daemon-reload && sudo systemctl restart docker`

Run the following script in `isaac_ros_common` to build the image and launch the container:

`$ scripts/run_dev.sh <optional path>`

You can either provide an optional path to mirror in your host ROS workspace with Isaac ROS packages, which will be made available in the container as `/workspaces/isaac_ros-dev`, or you can set up a new workspace in the container.


# Guided Tutorial: Nvblox with Nav2 and Isaac Sim
In this example, we will use nvblox to build a reconstruction from simulation data streamed from [Isaac Sim](https://developer.nvidia.com/isaac-sim). Data will flow from the simulator to nvblox using ROS2 and the ``isaac_ros_nvblox`` interface.

There are two ways to run nvblox in this example:
* Inside a Docker container
* In a ROS2 workspace installed directly on your machine

This example treats running docker as the default choice; running from native installation is described [further below in Additional Notes](#ros2-native-build).

## Example Description
In this example, Isaac Sim will run natively on your machine and communicate with nvblox running inside a Docker container. Running in Isaac Sim is referred to as running on the *host* machine, differentiating it from running inside the *Docker*. If using the native setup, both will run on the host machine. 

## Isaac Sim Setup (Host Machine)
Follow the standard instructions to install [Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_basic.html)
on the host machine.

As part of the set-up, make sure to install a local Nucleus server (Nucleus manages simulation assets such as maps and objects), following the instructions [here](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_faq.html#nucleus-and-cache). Mounting the Isaac share will give you access to the latest Isaac Sim samples, which these instructions use. Please also use the [latest URL for the mount](https://forums.developer.nvidia.com/t/known-issue-error-checking-isaac-sim-assets/204522) (rather than what's listed in the linked tutorial):

```
Name: Isaac
Type: Amazon S3
Host: d28dzv1nop4bat.cloudfront.net
Service: s3
Redirection: https://d28dzv1nop4bat.cloudfront.net
```

You will launch Isaac Sim from Python scripts that automate the setup of the robot and environment. Isaac Sim uses its own python binary,
which pulls in the modules that are dependencies. To run the Isaac Sim simulation launch scripts, you will use the Isaac Sim Python binary,
which is located at `~/.local/share/ov/pkg/{YOUR_ISAAC_SIM_VERSION}/python.sh`

For convenience, you can create an alias to this Python binary in your `~/.bashrc`. Using the Isaac Sim version `isaac_sim-2021.2.1-release.1`
as an example, add the following line to `~/.bashrc`
```
alias omni_python='~/.local/share/ov/pkg/isaac_sim-2021.2.1-release.1/python.sh'
```
**Note**: Ensure `isaac_sim-2021.2.1-release.1` is the name of the Isaac Sim version installed on your system.

Now ``source`` the `.bashrc` to have access to this alias.
```
source ~/.bashrc
```

## ROS2 Setup
The following instructions perform a Docker-based build. For a native build, follow the instructions [below](#ros2-native-build) instead.

### Isaac ROS Docker Setup

Clone the `isaac_ros_common` repo into a folder called `isaac_ros-dev`.

```
mkdir -p ~/workspaces/isaac_ros-dev/ros_ws/src
cd ~/workspaces/isaac_ros-dev/ros_ws/src
git clone --recurse-submodules https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
```

Follow the guide on the link to set up the `isaac_ros-dev` docker: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common

Start the Docker instance by navigating to the installed workspace and running the start script:

```
~/workspaces/isaac_ros-dev/ros_ws/src/isaac_ros_common/scripts/run_dev.sh
```

Install the dependencies for your ROS workspace:

```
cd /workspaces/isaac_ros-dev/ros_ws
rosdep install -i -r --from-paths src --rosdistro foxy -y --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv"
```

To build the code, first navigate to `/workspaces/isaac_ros-dev/ros_ws` inside the Docker container, then use the following command:

```
colcon build --packages-up-to nvblox_nav2 nvblox_ros nvblox_msgs nvblox_rviz_plugin
```

All tests should pass.

## Running the Simulation (on the Host) and the Reconstruction (in the Docker)
For this example, you will need two terminals. In the first terminal, you will run Isaac Sim.

**Note**: Ensure there is no ROS workspace sourced in this terminal.

**Terminal 1**: Start up Isaac Sim with the correct sensors on the host machine:

```
omni_python ~/workspaces/isaac_ros-dev/ros_ws/src/isaac_ros_nvblox/nvblox_isaac_sim/omniverse_scripts/carter_warehouse.py
```

**Note**: If Isaac Sim reports not finding a Nucleus server, follow the instructions [here](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_basic.html#isaac-sim-first-run) to download the required assets.

**Terminal 2:** In another terminal, start the `isaac_ros-dev` Docker:

```
~/workspaces/isaac_ros-dev/scripts/run_dev.sh
```

Source the ``ros_ws``:

```
source /workspaces/isaac_ros-dev/ros_ws/install/setup.bash
```

Run nvblox (and ROS2 Nav2):

```
ros2 launch nvblox_nav2 carter_sim.launch.py
```

You should see the robot reconstructing a mesh, with a costmap overlaid on top. To give it a command, you can select "2D Goal Pose"
in the command window at the top and select a goal in the main window. You should then see the robot plan a green path toward the
goal and navigate there, both in rviz and in simulation.

![readme_nav2](/uploads/0845bcad4f3d9ab05bbba0dbe3a659d7/readme_nav2.gif)

# Additional Notes

## ROS2 Native Build
First, follow the [installation instructions for installing the dependencies of the the core library](https://github.com/nvidia-isaac/nvblox#install-dependencies).
**Note** that you only have to install the dependencies: ROS2 will build the nvblox library itself.

```
sudo apt-get install -y libgoogle-glog-dev libgtest-dev libgflags-dev python3-dev
cd /usr/src/googletest && sudo cmake . && sudo cmake --build . --target install
```

Additionally, you need [CUDA](https://developer.nvidia.com/cuda-downloads) version 10.2 - 11.5 installed and sourced. To make sure Linux finds CUDA on your machine, make sure something like the following is present in your `~/.bashrc`:

```
export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/lib:${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

Install ROS2 foxy using the [Debian instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

**Note**: Sourcing ROS2 in your workspace automatically (i.e., in your ``.bashrc``) will cause Isaac Sim to break. You will need to
create an alias instead:

```
alias source_ros2="source /opt/ros/foxy/setup.bash;source ~/ros_ws/install/local_setup.bash"
```

Check out the nvblox repo to a path like `~/ros_ws/src`:

```
mkdir -p ~/ros_ws/src
git clone --recurse-submodules https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox
```

Then, build the entire workspace:
```
cd ~/ros_ws/ && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

Again, we recommend creating an alias:
```
alias cn="sh -c 'cd ~/ros_ws/ && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release' --verbose" 
```

### Running with the Native Build
The running instructions then become:
```
source_ros2
ros2 launch nvblox_nav2 carter_sim.launch.py
```
and the instructions for running Isaac Sim are the same as in the [section](#ros2-native-build) above.


# Package Reference
In this section we give an overview of parameters, inputs, and outputs within this repository and their purpose and settings.

## nvblox_ros
The package centers around the `nvblox_node`, whose parameters, inputs, and outputs are described in detail here.


### Published Topics

**mesh** ``nvblox_msgs::msg::Mesh`` \
  A visualization topic showing the mesh produced from the TSDF in a form that can be seen in RViz using `nvblox_rviz_plugin`. Set ``max_mesh_update_hz`` to control its update rate.

**pointcloud** ``sensor_msgs::msg::PointCloud2`` \
  A pointcloud of the 2D ESDF (Euclidean Signed Distance Field), with intensity as the metric distance to the nearest obstacle. Set ``max_esdf_update_hz`` to control its update rate.

**map_slice** ``nvblox_msgs::msg::DistanceMapSlice`` \
  A 2D slice of the ESDF, to be consumed by `nvblox_nav2` package for interfacing with Nav2. Set ``max_esdf_update_hz`` to control its update rate.

### Subscribed Topics

**tf** ``tf2_msgs::msg::TFMessage`` \
  The pose of the sensor relative to the global frame is resolved through TF2. Please see the [ROS2 documentation](https://docs.ros.org/en/foxy/Tutorials/Tf2/Introduction-To-Tf2.html) for more details.
   
**depth/image** ``sensor_msgs::msg::Image`` \
  The input depth image to be integrated. Must be paired with a `camera_info` message below. Supports both floating-point (depth in meters) and uint16 (depth in millimeters, OpenNI format).

**depth/camera_info** ``sensor_msgs::msg::CameraInfo`` \
  Required topic along with the depth image; contains intrinsic calibration parameters of the depth camera.

**color/image** ``sensor_msgs::msg::Image`` \
  Optional input color image to be integrated. Must be paired with a `camera_info` message below. Only used to color the mesh.

**color/camera_info** ``sensor_msgs::msg::CameraInfo`` \
  Optional topic along with the color image above, contains intrinsics of the color camera.

### Services

**save_ply** ``std_srvs::srv::Empty`` \
  This service has an empty request and response. Calling this service will write a mesh to disk at the path specified by the `output_dir` parameter.

### Parameters

A summary of the user settable parameters. All parameters are listed as:

**Parameter** `Default` \
  Description.

#### General Parameters

**voxel_size** `0.05` \
  Voxel size (in meters) to use for the map.

**esdf** `true` \
  Whether to compute the ESDF (map of distances to the nearest obstacle).

**esdf_2d** `true` \
  Whether to compute the ESDF in 2D (true) or 3D (false).

**distance_slice** `true` \
  Whether to output a distance slice of the ESDF to be used for path planning. 

**mesh** `true` \
  Whether to output a mesh for visualization in rviz, to be used with `nvblox_rviz_plugin`.

**global_frame** `map` \
  The name of the TF frame to be used as the global frame.

**slice_height** `1.0` \
  The *output* slice height for the distance slice and ESDF pointcloud. Does not need to be within min and max height below. In units of meters.

**min_height** `0.0` \
  The minimum height, in meters, to consider obstacles part of the 2D ESDF slice.

**max_height** `1.0` \
  The maximum height, in meters, to consider obstacles part of the 2D ESDF slice.

**max_tsdf_update_hz** `10.0` \
  The maximum rate (in Hz) at which to integrate depth images into the TSDF. A value of 0.0 means there is no cap.

**max_color_update_hz** `5.0` \
  The maximum rate (in Hz) at which to integrate color images into the color layer. A value of 0.0 means there is no cap.

**max_mesh_update_hz** `5.0` \
  The maximum rate (in Hz) at which to update and color the mesh. A value of 0.0 means there is no cap.

**max_esdf_update_hz** `2.0` \
  The maximum rate (in Hz) at which to update the ESDF and output the distance slice. A value of 0.0 means there is no cap.


#### Integrator Settings
**tsdf_integrator_max_integration_distance_m** `10.0` \
  The maximum distance, in meters, to integrate the TSDF up to.

**tsdf_integrator_truncation_distance_vox** `4.0` \
  The truncation distance, in units of voxels, for the TSDF.

**tsdf_integrator_max_weight** `100.0` \
  Maximum weight for the TSDF. Setting this number higher will lead to higher-quality reconstructions but worse performance in dynamic scenes.

**mesh_integrator_min_weight** `1e-4` \
  Minimum weight of the TSDF to consider for inclusion in the mesh.

**mesh_integrator_weld_vertices** `false` \
  Whether to weld identical vertices together in the mesh. Currently reduces the number of vertices by a factor of 5x, but is quite slow so we do not recommend you use this setting.

**color_integrator_max_integration_distance_m** `10.0` \
  Maximum distance, in meters, to integrate the color up to.

**esdf_integrator_min_weight** `1e-4` \
  Minimum weight of the TSDF to consider for inclusion in the ESDF.

**esdf_integrator_min_site_distance_vox** `1.0` \
  Minimum distance to consider a voxel within a surface for the ESDF calculation.

**esdf_integrator_max_distance_m** `10.0` \
  Maximum distance to compute the ESDF up to, in meters.


## nvblox_nav2
`nvblox_nav2` consists of two parts: an `nvblox_costmap_layer`, which is a Nav2 costmap plugin, and launch files for running the set-up with Nav2 in the loop (described above).

### `nvblox_costmap_layer` Parameters
**nvblox_map_slice_topic** `/nvblox_node/map_slice` \
  Topic to listen for the slice (set via parameters to allow easy configuration from a parameter file).

**max_obstacle_distance** `1.0` \
  Maximum distance from the surface to consider something to have a collision cost, in meters. This is *NOT* in addition to the inflation distance, but the total.

**inflation_distance** `0.5` \
  Distance to inflate all obstacles by, in meters.

# Troubleshooting

## Reconstruction without the planner (troubleshooting)
If something isn't working, or as a quick sanity check, you can run reconstruction without the planner.

Use the `carter_sim.launch.py` launch file parameters `run\_rviz` and `run\_nav2` to turn on and off running rviz and nav2. To run the reconstruction without the planner (but with rviz), run the following:
```
ros2 launch nvblox_nav2 carter_sim.launch.py run_rviz:=true run_nav2:=false
```

Now, command the robot to spin in place. In another terminal, source your ROS2 workspace again and enter:
```
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.1}}'
```
You should see the robot spin in a circle and reconstruct its environment.


## Do you have a connection to Isaac Sim?
A quick way to check if ROS2 is communicating correctly with Isaac Sim is to check whether the depth images are being sent.

```
ros2 topic hz /left/depth
```

If this does not receive any messages, then something is wrong with ROS2's connection to Isaac Sim. Either the file hasn't been set up correctly (make sure to run `nvblox_isaac_sim/omniverse_scripts/carter_warehouse.py` rather than opening an OV scene manually), or the scene is paused, or there are different ROS2 Domain IDs at play.



# Updates

| Date       | Changes                               |
| ---------- | ------------------------------------- |
| 2022-03-21 | Initial version.                      |

