# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

###################
# Thruster Header #
###################

uint8 cmd

#######################
# Extended Status MSG #
#######################
uint8 speed_loop_enable_status
uint8 minute_tag
uint8 second_tag
uint8 microsecond_tick_tag
int16 prop_pwm
float32 tach_rpm
uint8 fault_status
uint8 reset_status
uint8 speed_loop_interval
uint8 tach_err_count
float32 voltage_volts
float32 current_amps
