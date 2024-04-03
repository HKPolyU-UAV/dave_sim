# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# Position
float64 shift_northing
float64 shift_easting

# Negate the shift so it behaves like MC when applied to nav
bool shift_like_mc
