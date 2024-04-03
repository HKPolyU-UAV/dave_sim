# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

#####################################################################
# Raw data

uint8 count
uint32 analog_input1
uint32 analog_input2
float64 pressure #dbar
float64[3] vel #m/s
int8[3] amp
int8[3] corr