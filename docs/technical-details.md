# Nvblox Technical Details

Nvblox builds the reconstructed map in the form of a Truncated Signed Distance Function (TSDF) stored in a 3D voxel grid. This approach is similar to 3D occupancy grid mapping approaches in which occupancy probabilities are stored at each voxel. However, TSDF-based approaches like nvblox store the (signed) distance to the closest surface at each voxel. The surface of the environment can then be extracted as the zero-level set of this voxelized function. Typically, TSDF-based reconstructions provide higher quality surface reconstructions.

In addition to their use in reconstruction, distance fields are also useful for path planning, because they provide an immediate means of checking whether potential future robot positions are in collision. The dual utility of distance functions for both reconstruction and planning motivates their use in nvblox (a reconstruction library for path planning).
