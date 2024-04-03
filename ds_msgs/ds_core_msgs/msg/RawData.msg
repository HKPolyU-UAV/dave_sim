#
# A sequence of bytes
#

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header


int8 DATA_OUT=0 
int8 DATA_IN=1

int8 data_direction

uint8[] data
