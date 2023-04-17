# Tutorial For Human Reconstruction with Realsense

<div align="center"><img src="../resources/realsense_nvblox_humans.gif"/></div>

In this tutorial we'll demonstrate how one can perform dynamic human reconstruction in nvblox using realsense data.
If you want to know how human reconstruction works behind the scenes refer to the [technical details](./technical-details.md#human-reconstruction).

## Setup *Isaac ROS Image Segmentation*

  > **Note**: This example including nvblox with human reconstruction on RealSense data is not intended to run on a Jetson platform yet. Stay tuned for updates.
<!-- Split blockquote -->
  > **Note**: Currently we recommend the heavier `PeopleSemSegNet` over the lighter `PeopleSemSegNet ShuffleSeg` model provided in [*Isaac ROS Image Segmentation*](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_segmentation) for better segmentation performance.

The following steps show you how to run `PeopleSemSegNet` in ROS.
Refer to [this](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_segmentation) readme to run the `PeopleSemSegNet ShuffleSeg` network.

1. Complete all steps of the [RealSense](tutorial-realsense.md) tutorial and make sure it is working fine.

2. Clone the segmentation repository and its dependencies under `~/workspaces/isaac_ros-dev/src`.

   ```bash
   cd ~/workspaces/isaac_ros-dev/src
   ```

   ```bash
   git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_segmentation
   ```

   ```bash
   git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference
   ```

   ```bash
   git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
   ```

3. Pull down a ROS Bag of sample data:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_image_segmentation && \
        git lfs pull -X "" -I "resources/rosbags/"
    ```

4. Launch the Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

5. Download the `PeopleSemSegNet` ETLT file and the`int8` inference mode cache file:

    ```bash
    mkdir -p /workspaces/isaac_ros-dev/models/peoplesemsegnet/1
    cd /workspaces/isaac_ros-dev/models/peoplesemsegnet
    wget 'https://api.ngc.nvidia.com/v2/models/nvidia/tao/peoplesemsegnet/versions/deployable_quantized_vanilla_unet_v2.0/files/peoplesemsegnet_vanilla_unet_dynamic_etlt_int8.cache'
    wget 'https://api.ngc.nvidia.com/v2/models/nvidia/tao/peoplesemsegnet/versions/deployable_quantized_vanilla_unet_v2.0/files/peoplesemsegnet_vanilla_unet_dynamic_etlt_int8_fp16.etlt'
    ```

6. Convert the ETLT file to a TensorRT plan file:

    ```bash
    /opt/nvidia/tao/tao-converter -k tlt_encode -d 3,544,960 -p input_1:0,1x3x544x960,1x3x544x960,1x3x544x960 -t int8 -c peoplesemsegnet_vanilla_unet_dynamic_etlt_int8.cache -e /workspaces/isaac_ros-dev/models/peoplesemsegnet/1/model.plan -o argmax_1 peoplesemsegnet_vanilla_unet_dynamic_etlt_int8_fp16.etlt
    ```

7. Create the triton configuration file called `/workspaces/isaac_ros-dev/models/peoplesemsegnet/config.pbtxt` with the following content:

    ```bash
    name: "peoplesemsegnet"
    platform: "tensorrt_plan"
    max_batch_size: 0
    input [
      {
        name: "input_1:0"
        data_type: TYPE_FP32
        dims: [ 1, 3, 544, 960 ]
      }
    ]
    output [
      {
        name: "argmax_1"
        data_type: TYPE_INT32
        dims: [ 1, 544, 960, 1 ]
      }
    ]
    version_policy: {
      specific {
        versions: [ 1 ]
      }
    }
    ```

8. Inside the container, build and source the workspace:

    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

9. (Optional) Run tests to verify complete and correct installation:

    ```bash
    colcon test --executor sequential
    ```

10. Run this following launch file to get the ROS node running:

    ```bash
    ros2 launch isaac_ros_unet isaac_ros_unet_triton.launch.py model_name:=peoplesemsegnet model_repository_paths:=['/workspaces/isaac_ros-dev/models'] input_binding_names:=['input_1:0'] output_binding_names:=['argmax_1'] network_output_type:='argmax'
    ```

11. Open *two* other terminals, and enter the Docker container in both:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

12. Play the ROS bag in one of the terminals:

    ```bash
    ros2 bag play -l src/isaac_ros_image_segmentation/resources/rosbags/unet_sample_data/
    ```

    And visualize the output in the other terminal:

      ```bash
      ros2 run rqt_image_view rqt_image_view
      ```

13. Verify that the output looks similar to [this image.](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_segmentation/blob/main/resources/peoplesemsegnet_shuffleseg_rqt.png)

## Example with Realsense Live Data

1. Complete the [*Isaac ROS Image Segmentation* setup](#setup-isaac-ros-image-segmentation) above.

2. Connect the RealSense device to your machine.

3. Run the ROS Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

4. Source the workspace:

    ```bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    ```

5. At this point, you can check that the RealSense camera is connected by running realsense-viewer:

    ```bash
    realsense-viewer
    ```

6. If successful, run the launch file to spin up the example:

    ```bash
    ros2 launch nvblox_examples_bringup realsense_humans_example.launch.py
    ```

## Example with Realsense Recorded Data

Refer to the [RealSense recording tutorial](./tutorial-realsense-record.md) on how to record RealSense data.
Below we show how to run the example on your own recorded ROS bags.

1. Complete the [*Isaac ROS Image Segmentation* setup](#setup-isaac-ros-image-segmentation) above.

2. Run the ROS Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

3. Source the workspace:

    ```bash
    source /workspaces/isaac_ros-dev/install/setup.bash
    ```

4. If successful, run the launch file to spin up the example:

    ```bash
    ros2 launch nvblox_examples_bringup realsense_humans_example.launch.py from_bag:=True bag_path:=<PATH_TO_YOUR_BAG>
    ```

## Troubleshooting

See our troubleshooting page [here](troubleshooting-nvblox-realsense.md).
