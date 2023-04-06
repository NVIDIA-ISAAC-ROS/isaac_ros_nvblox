# Nvblox Technical Details

Nvblox builds the reconstructed map in the form of a Truncated Signed Distance Function (TSDF) stored in a 3D voxel grid. This approach is similar to 3D occupancy grid mapping approaches in which occupancy probabilities are stored at each voxel. However, TSDF-based approaches like nvblox store the (signed) distance to the closest surface at each voxel. The surface of the environment can then be extracted as the zero-level set of this voxelized function. Typically, TSDF-based reconstructions provide higher quality surface reconstructions.

In addition to their use in reconstruction, distance fields are also useful for path planning, because they provide an immediate means of checking whether potential future robot positions are in collision. The dual utility of distance functions for both reconstruction and planning motivates their use in nvblox (a reconstruction library for path planning).

<div align="center"><img src="../resources/system_diagram_technical_details.jpg"/></div>

The diagram above indicates data and processes in nvblox. In the default configuration nvblox builds TSDF, color, mesh, and ESDF *layers*. Each *layer* is a independent, but aligned and co-located, voxel grid containing data of the appropriate type. For example, voxels on the TSDF layer store distance and weight data, while the color layer voxels store color values.

# Human Reconstruction

Additionally to the static TSDF mapping we also provide a human reconstruction and dynamic mapping utility using occupancy grid mapping.

Human segmentation is applied to each processed color frame with [*Isaac ROS Image Segmentation*](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_segmentation). The depth masker module uses the segmentation mask from the color image to separate the depth-map into human and non-human parts. While the non-human labelled part of the depth frame is still forwarded to TSDF mapping as explained above, the human labelled part is processed to an occupancy grid map. 

To relax the assumption that occupancy grid maps only capture static objects we apply an occupancy decay step. At a fixed frequency, all voxel occupancy probabilities are decayed towards `0.5` over time. This means that the state of the map (occupied or free) becomes less certain after it has fallen out of the field of view, until it becomes unknown (`0.5` occupancy probability).

<div align="center"><img src="../resources/system_diagram_humans.png"/></div>
