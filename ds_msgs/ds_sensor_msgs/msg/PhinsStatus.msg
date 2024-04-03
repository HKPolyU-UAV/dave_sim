# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# Algorithm status flags
ds_nmea_msgs/PixseAlgsts algorithm

# Sensor status flags
ds_nmea_msgs/PixseStatus system

# User status flags.
ds_nmea_msgs/PixseHtsts  user
