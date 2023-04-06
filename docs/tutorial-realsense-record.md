# Tutorial For Realsense Data Recording

## Record ROS Bag

To record realsense data for nvblox:

1. Make sure that all [realsense dependencies](./tutorial-realsense.md) are met.

2. Connect the camera, start the docker container and source the workspace as explained in the [realsense example tutorial](./tutorial-realsense.md).

3. Start recording:

    ```bash
    ros2 launch nvblox_examples_bringup record_realsense.launch.py
    ```

4. Stop the recording when done.

## Start NVBLOX from ROS Bag

1. Start nvblox from bag:

    ```bash
    ros2 launch nvblox_examples_bringup realsense(_humans)_example.launch.py from_bag:=True bag_path:=<path_to_recorded_bag>
    ```
