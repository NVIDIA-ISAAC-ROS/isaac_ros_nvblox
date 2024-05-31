# A distance map slice is effectively a floating point image where the 
# value of each pixel represents the distance in meters to the closest
# surface, in 2 dimensions. Pixels covering space which have not been
# observed in the mapping process are given a sentinal value which
# indicates their unknown state.

std_msgs/Header header

# How big each "pixel" in the map is in meters.
# In other words the side length of each cell.
float32 resolution

# Width is along the x axis and height is along the y axis.
uint32 width
uint32 height

# The location of the origin of the distance map slice image.
# This point coincides with the upper-left corner of the upper-left most pixel.
# The distance map slice is effectively an axis-aligned image, attached to
# this point in the world frame.
geometry_msgs/Point origin

# Which value is used for "unknown" cells.
# These cells have not been observed by the camera during the mapping process.
float32 unknown_value

# The flattened 2D costmap data. Data is stored as floating point values in meters.
# The data is stored row-major such that each row of the image/slice sits contiguous
# in memory.
float32[] data
