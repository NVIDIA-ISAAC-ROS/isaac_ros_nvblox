# Parameters

This page contains all parameters exposed to ROS 2.
For the provided examples in the `nvblox_example_bringup` package, parameters
are set using YAML configuration files under `nvblox_examples/nvblox_examples_bringup/config`.

The base parameters are set in `nvblox_base.yaml` and there exist multiple specialization files that are overwriting parameters from the base file depending on which example is launched.

File tree:
```bash
config
├── nvblox
    ├── nvblox_base.yaml
    └── specializations
        ├── nvblox_humans.yaml
        ├── nvblox_isaac_sim.yaml
        └── nvblox_realsense.yaml
```

Loaded specializations for each example:
| Launch file                        | Specializations                               |
| ---------------------------------- | --------------------------------------------- |
| isaac_sim_example.launch.py        | `nvblox_isaac_sim.yaml`                       |
| isaac_sim_humans_example.launch.py | `nvblox_isaac_sim.yaml`, `nvblox_humans.yaml` |
| realsense_example.launch.py        | `nvblox_realsense.yaml`                       |
| realsense_humans_example.launch.py | `nvblox_realsense.yaml`,`nvblox_humans.yaml`  |

## General Parameters

| ROS Parameter                             | Type     | Default                   | Description                                                                                                                                                                                                        |
| ----------------------------------------- | -------- | ------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `global_frame`                            | `string` | `odom`                    | The name of the TF frame to be used as the global frame. For the realsense examples, this parameter is exposed as a launch argument.                                                                               |
| `voxel_size`                              | `float`  | `0.05`                    | Voxel size (in meters) to use for the map.                                                                                                                                                                         |
| `use_tf_transforms`                       | `bool`   | `true`                    | Whether to use TF transforms at all. If set to false, `use_topic_transforms` must be true and `pose_frame` *needs* to be set.                                                                                      |
| `use_topic_transforms`                    | `bool`   | `false`                   | Whether to listen to topics for transforms. If set to true, will try to get `global_frame` to `pose_frame` transform from the topics. If set to false, everything will be resolved through TF.                     |
| `compute_esdf`                            | `bool`   | `true`                    | Whether to compute the ESDF (map of distances to the nearest obstacle).                                                                                                                                            |
| `esdf_update_rate_hz`                     | `float`  | `2.0`                     | The rate (in Hz) at which to update the ESDF and output the distance slice.                                                                                                                                        |
| `esdf_2d`                                 | `bool`   | `true`                    | Whether to compute the ESDF in 2D (true) or 3D (false).                                                                                                                                                            |
| `esdf_distance_slice`                     | `bool`   | `true`                    | Whether to output a distance slice of the ESDF to be used for path planning.                                                                                                                                       |
| `esdf_slice_height`                       | `float`  | `1.0`                     | The *output* slice height for the distance slice and ESDF pointcloud. Does not need to be within min and max height below. In units of meters.                                                                     |
| `esdf_2d_min_height`                      | `float`  | `0.0`                     | The minimum height, in meters, to consider obstacles part of the 2D ESDF slice.                                                                                                                                    |
| `esdf_2d_max_height`                      | `float`  | `1.0`                     | The maximum height, in meters, to consider obstacles part of the 2D ESDF slice.                                                                                                                                    |
| `compute_mesh`                            | `bool`   | `true`                    | Whether to output a mesh for visualization in rviz, to be used with `nvblox_rviz_plugin`.                                                                                                                          |
| `mesh_update_rate_hz`                     | `float`  | `5.0`                     | The rate (in Hz) at which to update and publish the mesh.                                                                                                                                                          |
| `use_color`                               | `bool`   | `true`                    | Whether to integrate color images to color the mesh.                                                                                                                                                               |
| `max_color_update_hz`                     | `float`  | `5.0`                     | The maximum rate (in Hz) at which to integrate color images into the color layer. A value of 0.0 means there is no cap.                                                                                            |
| `use_depth`                               | `bool`   | `true`                    | Whether to integrate depth images.                                                                                                                                                                                 |
| `max_depth_update_hz`                     | `float`  | `10.0`                    | The maximum rate (in Hz) at which to integrate depth images. A value of 0.0 means there is no cap.                                                                                                                 |
| `use_lidar`                               | `bool`   | `true`                    | Whether to integrate LiDAR scans.                                                                                                                                                                                  |
| `max_lidar_update_hz`                     | `float`  | `10.0`                    | The maximum rate (in Hz) at which to integrate LiDAR scans. A value of 0.0 means there is no cap.                                                                                                                  |
| `lidar_width`                             | `int`    | `1800`                    | Width of the LIDAR scan, in number of beams. Defaults are for the Velodyne VLP16.                                                                                                                                  |
| `lidar_height`                            | `int`    | `16`                      | Height of the LIDAR scan, in number of beams. Defaults are for the VLP16.                                                                                                                                          |
| `lidar_vertical_fov_rad`                  | `float`  | `30 degrees (in radians)` | The vertical field of view of the LIDAR scan, in radians. Horizontal FoV is assumed to be 360 degrees. This is used to calculate the individual beam angle offsets.                                                |
| `use_static_occupancy_layer`              | `float`  | `false`                   | Whether to use the static occupancy layer for projective integration. If this flag is set to false (default), TSDF integration is used.                                                                            |
| `occupancy_publication_rate_hz`           | `float`  | `2.0`                     | The rate (in Hz) at which to publish the static occupancy pointcloud.                                                                                                                                              |
| `max_poll_rate_hz`                        | `float`  | `100.0`                   | Specifies what rate to poll the color & depth updates at. Will exit as no-op if no new images. Set this higher than you expect images to come in at.                                                               |
| `maximum_sensor_message_queue_length`     | `int`    | `30`                      | How many messages to store in the sensor messages queues (depth, color, lidar) before deleting oldest messages.                                                                                                    |
| `map_clearing_radius_m`                   | `float`  | `-1.0`                    | Radius around the `map_clearing_frame_id` outside which we clear the map. Note that values <= 0.0 indicate that no clearing is performed.                                                                          |
| `map_clearing_frame_id`                   | `string` | `base_link`               | The name of the TF frame around which we clear the map.                                                                                                                                                            |
| `clear_outside_radius_rate_hz`            | `float`  | `1.0`                     | The rate (in Hz) at wich we clear the map outside of the `map_clearing_radius_m`.                                                                                                                                  |
| `depth_qos`                               | `string` | `SYSTEM_DEFAULT`          | ROS 2 QoS string for the depth subscription.                                                                                                                                                                       |
| `color_qos`                               | `string` | `SYSTEM_DEFAULT`          | ROS 2 QoS string for the color subscription.                                                                                                                                                                       |
| `pose_frame`                              | `float`  | `base_link`               | Only used if `use_topic_transforms` is set to true. Pose and transform messages will be interpreted as being in this pose frame, and the remaining transform to the sensor frame will be looked up on the TF tree. |
| `slice_visualization_attachment_frame_id` | `string` | `base_link`               | Frame to which the map slice bounds visualization is centered on the xy-plane.                                                                                                                                     |
| `slice_visualization_side_length`         | `float`  | `10.0`                    | Side length of the map slice bounds visualization plane.                                                                                                                                                           |

# Mapper Parameters

The nvblox node holds a mapper object that takes care of updating all the layers.
Depending on the `use_static_occupancy_layer` parameter above, it builds either a tsdf or occupancy probability grid.
Below is a list of all the parameters to set up this mapper.

| ROS Parameter                                                   | Type     | Default          | Description                                                                                                                                        |
| --------------------------------------------------------------- | -------- | ---------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| `mapper.projective_integrator_max_integration_distance_m`       | `float`  | `7.0`            | The maximum distance, in meters, to integrate the TSDF or occupancy map up to for depth images.                                                    |
| `mapper.projective_integrator_truncation_distance_vox`          | `float`  | `4.0`            | The truncation distance, in units of voxels, for the TSDF or occupancy map.                                                                        |
| `mapper.lidar_projective_integrator_max_integration_distance_m` | `float`  | `10.0`           | The maximum distance, in meters, to integrate the TSDF or occupancy map up to for LiDAR scans.                                                     |
| `mapper.weighting_mode`                                         | `string` | `inverse_square` | The weighting mode, applied to TSDF and color integrations. Options: [`constant`, `constant_dropoff`, `inverse_square`, `inverse_square_dropoff`]. |
| `mapper.tsdf_integrator_max_weight`                             | `float`  | `100.0`          | Maximum weight for the TSDF. Setting this number higher will lead to higher-quality reconstructions but worse performance in dynamic scenes.       |
| `mapper.free_region_occupancy_probability`                      | `float`  | `0.3`            | The inverse sensor model occupancy probability for voxels observed as free space.                                                                  |
| `mapper.occupied_region_occupancy_probability`                  | `float`  | `0.7`            | The inverse sensor model occupancy probability for voxels observed as occupied.                                                                    |
| `mapper.unobserved_region_occupancy_probability`                | `float`  | `0.5`            | The inverse sensor model occupancy probability for unobserved voxels.                                                                              |
| `mapper.occupied_region_half_width_m`                           | `float`  | `0.1`            | Half the width of the region which is considered as occupied.                                                                                      |
| `mapper.esdf_integrator_min_weight`                             | `float`  | `1e-4`           | Minimum weight of the TSDF to consider for inclusion in the ESDF.                                                                                  |
| `mapper.esdf_integrator_max_distance_m`                         | `float`  | `2.0`            | Maximum distance to compute the ESDF up to, in meters.                                                                                             |
| `mapper.esdf_integrator_max_site_distance_vox`                  | `float`  | `1.0`            | Maximum distance to consider a voxel within a surface for the ESDF calculation.                                                                    |
| `mapper.mesh_integrator_min_weight`                             | `float`  | `1e-4`           | Minimum weight of the TSDF to consider for inclusion in the mesh.                                                                                  |
| `mapper.mesh_integrator_weld_vertices`                          | `bool`   | `true`           | Whether to weld identical vertices together in the mesh.                                                                                           |
| `mapper.color_integrator_max_integration_distance_m`            | `float`  | `7.0`            | Maximum distance, in meters, to integrate the color up to.                                                                                         |

# Human Reconstruction Parameters

In case of the `nvblox_human_node`, there are two individual mappers (contained in a multi-mapper object) taking care of updating the layers.
The first mapper is equivalent to the mapper of the `nvblox_node` explained above and is also initialized with the same parameters.

The second mapper handles dynamic reconstruction of humans in an occupancy layer.
Using the human occupancy layer an esdf layer and costmap are generated.
The parameters of the human mapper for occupancy and esdf integration are duplicates of the parameters for the static mapper.
Only the parent parameter name needs to be changed to `human_mapper` (i.e. `mapper.esdf_integrator_min_weight` -> `human_mapper.esdf_integrator_min_weight`).

Below you find additional parameters that only exist for the 'nvblox_human_node'.

| ROS Parameter                                    | Type    | Default | Description                                                                                                        |
| ------------------------------------------------ | ------- | ------- | ------------------------------------------------------------------------------------------------------------------ |
| `human_esdf_update_rate_hz`                      | `float` | `10.0`  | The rate (in Hz) at which to update the human ESDF, output the distance slice and additional human visualizations. |
| `human_occupancy_decay_rate_hz`                  | `float` | `10.0`  | The rate (in Hz) at which to decay the human occupancy layer.                                                      |
| `human_mapper.free_region_decay_probability`     | `float` | `0.55`  | The decay probability that is applied to the free region on decay. Must be in `[0.5, 1.0]`.                        |
| `human_mapper.occupied_region_decay_probability` | `float` | `0.4`   | The decay probability that is applied to the occupied region on decay. Must be in `[0.0, 0.5]`.                    |

> **Note**: Decay of the occupancy layer is needed to handle dynamic objects like humans. Without the decay observed dynamic objects stay in the map even if not seen for a long time period. In that time the object might have moved and the map is therefore invalid. Using the decay we simulate the loss of knowledge about parts of the dynamic map currently not observed over time.
