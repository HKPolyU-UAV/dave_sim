# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

string servo_name

# There are case where we might not actually know the current state
float32 SERVO_ANGLE_UNKNOWN = -9999.9

bool enable
float32 actual_radians
float32 cmd_radians

