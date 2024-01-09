# Static Reconstruction with Hawk Camera

This tutorial demonstrates how to perform 3D reconstruction using a [Hawk Stereo Camera](https://leopardimaging.com/leopard-imaging-hawk-stereo-camera/), [Isaac ROS DNN Stereo Depth](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth), [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) and [Isaac ROS Nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox) on a [Nova Carter](https://nvidia-isaac-ros.github.io/robots/nova_carter.html).

## Foxglove setup

> **_NOTE:_** These steps must be done on your host computer.

1. Install Foxglove Studio from [here](https://foxglove.dev/download)

2. Install the nvblox mesh foxglove extension following [this readme](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/blob/release-3.0-prerelease/nvblox_foxglove/README.md).

> **_NOTE:_** The extension is needed to visualize the nvblox mesh published on the `/nvblox_node/mesh` topic.

3. To test that the extension works, please run the pipeline (as described [below](#running-the-pipeline)) and check that a mesh is visualized on the `/nvblox_node/mesh` topic.

## Nova Carter Setup

Before proceeding to the next section, make sure to set up your Nova Carter robot according to the [Nova Carter Guide](https://nvidia-isaac-ros.github.io/robots/nova_carter.html) and test that the given [Demo Applications](https://nvidia-isaac-ros.github.io/robots/nova_carter.html#demo-applications) work.

## Installing the Dependencies

1. Checkout the `isaac_ros_nvblox` repository to the `release-3.0-prerelease` branch:

```bash
cd $ISAAC_ROS_WS/src/isaac_ros_nvblox && \
  git checkout release-3.0-prerelease
```

2. Clone the missing dependency repositories using the `vcstool` file in the `isaac_ros_nvblox` repository:

```bash
vcs import --recursive $ISAAC_ROS_WS/src < $ISAAC_ROS_WS/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/hawk_example.repos
```
> **_NOTE:_** After these steps, `isaac_ros_nvblox` should be checked out on the `release-3.0-prerelease` branch and all other dependency repositories on the `release-2.1` branch.

3. Copy the ess engine file that was created during steps 5 & 6 of the [Repository Setup](https://nvidia-isaac-ros.github.io/robots/nova_carter.html#repository-setup) to the `$ISAAC_ROS_WS/models` folder:

```bash
mkdir $ISAAC_ROS_WS/models && \
cp $ISAAC_ROS_WS/src/nova_carter/carter_navigation/models/ess.engine $ISAAC_ROS_WS/models
```

## Running the Pipeline

1. Connect the joystick by following steps 1-2 from [Joystick Driving with 4 Owl Fisheye Cameras](https://nvidia-isaac-ros.github.io/robots/nova_carter.html#demo-applications).

2. Open a new terminal and [connect it to the robot](https://nvidia-isaac-ros.github.io/robots/nova_carter.html#connecting-the-pc-to-the-robot).

3. Launch the Docker container using the `run_dev.sh` script:

```bash
cd $ISAAC_ROS_WS/src/isaac_ros_common && \
  ./scripts/run_dev.sh $ISAAC_ROS_WS
```

4. Build the ROS packages in the Docker container:

```bash
cd /workspaces/isaac_ros-dev && \
   colcon build --symlink-install --continue-on-error
```

5. Run the launch file:

```bash
cd /workspaces/isaac_ros-dev && \
   source install/setup.bash && \
   ros2 launch nvblox_examples_bringup hawk_example.launch.py
```

6. On your visualization computer (laptop), follow steps 4-6 of the [Foxglove Setup](https://nvidia-isaac-ros.github.io/robots/nova_carter.html#foxglove-setup). The `hawk_example_foxglove.json` layout file to visualize the example can be found in the [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/blob/release-3.0-prerelease/nvblox_examples/nvblox_examples_bringup/config/visualization/hawk_example_foxglove.json).

7. Move the robot using the joystick. Controls can be found in step 8 of the [Joystick Driving with 4 Owl Fisheye Cameras](https://nvidia-isaac-ros.github.io/robots/nova_carter.html#demo-applications) demo.

You should now see a mesh, an esdf pointcloud (costmap), as well as depth and color images:

![Hawk Example GIF](hawk_example.gif)
