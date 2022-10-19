std_msgs/Header header

# How big each "pixel" in the map is.
float32 resolution

# Width is along the x axis and height is along the y axis.
uint32 width
uint32 height

# The location of the "origin" (upper left) of the image. This refers to
# the "upper-left" of the upper left most pixel as well.
# TODO(helen): In the future, potentially switch to pose instead.
geometry_msgs/Point origin

# Which value is used for "unknown" cells.
float32 unknown_value

# Data is stored as floats since it's a distance map.
float32[] data
