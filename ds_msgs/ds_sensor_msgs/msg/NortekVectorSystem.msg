# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

#####################################################################
# Raw data

time clk
float64 battery #V
float64 sound_speed #m/s
float64 heading #deg
float64 pitch #deg
float64 roll #deg
float64 temperature #degC
uint8 error
uint8 status
uint16 analog_input