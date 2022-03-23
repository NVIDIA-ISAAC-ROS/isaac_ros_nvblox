std_msgs/Header header

# Block origins and block sizes must match.
# Block size is the physical size (in meters) of a block of the mesh.
float32 block_size
# Block indices are the 3D indices of the blocks; to get the origin of the
# block, simply multiply its index by the block size.
# Note that we consider a block's origin to be the low-side corner of
# the low-side voxel.
Index3D[] block_indices
# Blocks contains the actual mesh within the block, including triangles,
# vertex positions, and vertex colors.
MeshBlock[] blocks

# Whether to clear the entire previous map. This is set to true when
# the *entire* map rather than only the pieces that changed are published.
bool clear