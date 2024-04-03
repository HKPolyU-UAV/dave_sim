# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

float64 lat
float64 lon

float64 northing
float64 easting
float64 down
float64 roll
float64 pitch
float64 heading

float64 surge_u
float64 sway_v
float64 heave_w
float64 p
float64 q
float64 r

geometry_msgs/Quaternion orientation