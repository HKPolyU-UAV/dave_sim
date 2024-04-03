# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# Gain parameters
float64 kp
float64 ki
float64 kd

# Min and max limits for the integrated error term
float64 min_integrated_error
float64 max_integrated_error

# Min and max limits for the PID controller output
float64 min_output_saturation
float64 max_output_saturation
