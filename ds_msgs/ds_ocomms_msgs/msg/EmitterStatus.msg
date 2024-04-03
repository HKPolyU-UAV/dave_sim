# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

string source
uint16 count
uint16 addr
uint16 enable_val
uint16 dim_val
uint16 temp_1
uint16 temp_2 