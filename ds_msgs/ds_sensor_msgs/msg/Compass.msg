# Message for a Compass

float64 COMPASS_NO_DATA = -9999.0

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# In degrees.
float64 heading            # + turns to east
float64 heading_covar

bool is_true_heading       # true: true heading, false: magnetic heading
