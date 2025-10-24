# VoxelSerialized is a simpler representation of the Marker message. It is used for simpler
# processing where the information is contained in a single continuous array that can be directly
# copied to the GPU for rendering.

std_msgs/Header header

# Voxel size in meters
float32 voxel_size_m
# Number of voxels, given for convenience as it is equal to points.size()/3
int32 num_voxels
# Array reprensenting the centers of each voxel [x0, y0, z0, x1, y1, z1, ...]
float32[] points
# Array reprensenting the color of each voxel [r0, g0, b0, a0, r1, g1, b1, a1, ...]
float32[] colors
