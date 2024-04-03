# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

RovAutoState autos
bool enabled

control_msgs/PidState fwd_state
control_msgs/PidState stbd_state
control_msgs/PidState down_state
control_msgs/PidState heading_state
