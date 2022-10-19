# Troubleshoot for RealSense-based Reconstruction Example

This page contains troubleshooting for the nvblox-VSLAM-realsense example found [here](tutorial-nvblox-vslam-realsense.md). The example combines nvblox-based reconstruction with VSLAM. Both of these processes, nvblox and VSLAM, are driven by the realsense depth camera.

The setup for the example is shown below.

<div align="center"><img src="../resources/nvblox_realsense_example_diagram.png"/></div>

Images (2 x IR images and 1 x Depth image) are published at 60Hz from the realsense ROS2 node. At the time of publishing we are using the `ros2-beta` branch of [realsense-ros](https://github.com/IntelRealSense/realsense-ros/tree/ros2-beta). This is the branch used by the ROS2 apt packages (viewable on the [ros package index](https://index.ros.org/p/realsense2_camera/github-IntelRealSense-realsense-ros/#humble)).

> **Why do we include the realsense splitter?**: The depth output from the realsense is significantly improved when using its inbuilt projector which increases the amount of texture in the scene. However, VSLAM performance is disrupted by the projector. We, therefore, use the realsense's toggling feature such that the projector is switched on and off in interleaved frames (on, off, on, off, on, off,...etc.). The splitter splits this interleaved image stream into two channels, one with the projector on and one with the projector off. IR images with the projector off are passed to VSLAM, while depth images with the projector on are passed to nvblox. This splitting occurs based on the metadata associated with each frame, published by the realsense, which contains a field which states if the projector was on at the time of frame capture.

## Symptoms

No reconstruction appears in RVIZ, and the terminal reports:

```bash
[nvblox_node-3] Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[nvblox_node-3] at line 93 in /opt/ros/humble/src/geometry2/tf2/src/buffer_core.cpp
[nvblox_node-3] Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[nvblox_node-3] at line 93 in /opt/ros/humble/src/geometry2/tf2/src/buffer_core.cpp
```

**Explanation:** This message is repeating because nvblox can't get the camera pose in the odom frame. It can't get the pose because VSLAM is not publishing it. VSLAM is not publishing anything because (likely) it is not receiving input data.

So the issue here is that VSLAM is not getting data. This is caused either by one of:
1) Images not coming off the camera into Linux.
2) Images not being published by the ROS node.
3) Images not making it through the splitter (usually because the metadata does not contain the projector state).

I have seen all three of these issues. All three of them have the same symptom (no pose for nvblox), but all three have different solutions.

## Issue

From what I can tell, the underlying issue is that the realsense does not support recent versions of the Linux kernel (see [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) for the list of supported kernel versions). There are some related issues [here](https://github.com/IntelRealSense/librealsense/issues/10439) and [here](https://github.com/IntelRealSense/librealsense/issues/5229).

To determine what to do we have to do a few checks.

### Check #1 - Images and Metadata are coming into Linux

Check that images are making it off the camera. Start the `realsense-viewer`.

```bash
realsense-viewer
```
Then:

1) Before turning on the "Stereo Module" click the drop down arrow **(red box below)** and turn on Infrared camera. 
2) Turn on the stereo module
3) Click the further drop down labels "controls" and click on the option "Emitter On Off". **(purple box below)**
4) In the upper right change from 3D view to 2D. **(green box below)**
5) You should see something like the image below.
6) On the greyscale image click the left-most icon in the upper-right-hand corner which looks like three horizontal bars with two tiny check marks. This displays the meta data. **(orange box below)**

Do you see two images like below?

<div align="center"><img src="../resources/realsense_troubleshooting_viewer_ir.png"/></div>

Do you see the metadata labelled "Frame Laser Pose Mode" **(green box above)** which should be flicking between 0 and 1 with the emitter. If no, preceded to the section below for [Solution 1 - Install DKMS](#solution-1---install-dkms)


### Check #2 - Check Images are Being Published in ROS

You're here because your realsense-viewer is operating as expected, i.e. is showing IR images and emitter-state metadata, but the nvblox, VSLAM, realsense example still isn't generating a reconstruction.

Start the example:

```bash
ros2 launch nvblox_examples_bringup nvblox_vslam_realsense.launch.py
```

In rviz check that the following images can be viewed.

- Topics before splitter
  - `/camera/color/image_raw`
  - `/camera/depth/image_rect_raw`
  - `/camera/infra1/image_rect_raw`
  - `/camera/infra2/image_rect_raw`
- Topics after splitter
  - `/camera/realsense_splitter_node/output/depth`
  - `/camera/realsense_splitter_node/output/infra_1`
  - `/camera/realsense_splitter_node/output/infra_2`

The topics can be viewed by changing the topic field of the image topic **(red box below)**

<div align="center"><img src="../resources/realsense_troubleshooting_rviz.png"/></div>

If the topics before the splitter are not all viewable, preceded to [Solution 2](#solution-2---build-without-cuda) - build realsense-ros without CUDA.

If only the topics after the splitter are not available and [Check 1](#check-1---images-and-metadata-are-coming-into-linux) passed. I'm out of ideas and you should probably get in contact.



## Solutions

The solutions I have found boil down to:

- Install the realsense DKMS, or
- Build without CUDA, or
- Both of the above.
 
Try the solution the above checks directed you to. Repeat the checks 1 and 2. If you still have no luck, attempt the other solution.

### Solution 1 - Install DKMS

You're here because either you get no images off the camera, or you don't get the correct metadata field.

On a supported kernel version, the apt package patches the kernel when you install `librealsense-dkms` such that the realsense operates as expected. 

If everything is as expected on a supported kernel and you run

```bash
helen@holeynikova-dt:~/workspaces/isaac_ros-dev/src/isaac_ros_common (dev)$ uname -r
5.8.0-43-generic

helen@holeynikova-dt:~/workspaces/isaac_ros-dev/src/isaac_ros_common (dev)$ dkms status
librealsense2-dkms, 1.3.18, 5.8.0-43-generic, x86_64: installed
librealsense2-dkms, 1.3.18, 5.8.0-67-generic, x86_64: installed
nvidia, 510.85.02, 5.13.0-40-generic, x86_64: installed
nvidia, 510.85.02, 5.8.0-43-generic, x86_64: installed

helen@holeynikova-dt:/lib/udev/rules.d$ modinfo uvcvideo | grep "version:"
version: 1.1.2.realsense-1.3.18
srcversion: 964E8D3335D17053B8EEDD2
```

The important thing to see here is that the word "realsense" appears:
- after the second command, it appears **next to your version of the kernel**, and
- after the third command, it appears at all.

Likely, if you're here, you have a recent kernel and the words realsense do not appear, or appear in the second command next to an old kernel version. Something like:

```bash
shubham@shubham-linuxbox:~$ uname -r
5.15.0-48-generic
shubham@shubham-linuxbox:~$ dkms status
nvidia, 510.85.02, 5.15.0-48-generic, x86_64: installed
shubham@shubham-linuxbox:~$ modinfo uvcvideo | grep "version:"
version:        1.1.1
srcversion:     18D809600E7D1E107042647
```

**The solution** is to install the dkms for recent versions of the kernel. We found these here:

https://github.com/mengyui/librealsense2-dkms/releases/tag/initial-support-for-kernel-5.15

Download the .deb file named `librealsense2-dkms-dkms_1.3.14_amd64.deb`
In the directory where you downloaded the file run: `sudo apt install ./librealsense2-dkms-dkms_1.3.14_amd64.deb`
Now the commands above, on my machine, return:

```bash
alex@shallowinsight:~$ uname -r
5.15.0-48-generic

alex@shallowinsight:~$ dkms status
librealsense2-dkms, 1.3.14, 5.15.0-48-generic, x86_64: installed
librealsense2-dkms, 1.3.14, 5.15.0-50-generic, x86_64: installed
nvidia, 510.85.02, 5.15.0-48-generic, x86_64: installed
nvidia, 510.85.02, 5.15.0-50-generic, x86_64: installed
nvidia, 510.85.02, 5.4.0-126-generic, x86_64: installed
nvidia, 510.85.02, 5.4.0-128-generic, x86_64: installed

alex@shallowinsight:~$ modinfo uvcvideo | grep "version:"
version:        1.1.2.realsense-1.3.14
srcversion:     26234508927E0F6886C9A48
```
**Go back and repeat check 1. You should have images + metadata.**


### Solution 2 - Build without CUDA

You're here because check 1 passed; you have images on your machine + metadata, but you get no images in ROS. The solution here is to build librealsense without CUDA.

Manually build librealsense within the container without CUDA. Run the following commands within the isaac_ros_common docker container:

```bash
/opt/realsense/build-librealsense.sh -n
```

now rebuild the ROS2 workspace

```bash
rm -rf build/ install/ log/
colcon build --symlink-install --continue-on-error
source install/setup.bash
```

**Go back and repeat check 2. You should have images in ROS before and after the splitter.**

# Things still aren't working
Well this isn't going well. One thing to try is to build with `-DFORCE_RSUSB_BACKEND=true` as reported in this [issue](https://github.com/IntelRealSense/librealsense/issues/10439).

Otherwise, file an issue, and describe the steps you've taken here.
