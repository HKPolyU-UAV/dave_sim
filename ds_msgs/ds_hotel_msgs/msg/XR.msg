# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

float64 deadhour
bool good
string address
uint16 idnum

#############
# code_status

uint8 C

################
# release_status

# valve system control word
uint8 V
uint8 valve_current_level
bool valve_run
bool valve_open
bool valve_battery_set_on
bool valve_battery_set_off

# motor release control word
uint8 M
bool motor_2_drop
bool motor_2_hold
bool motor_1_drop
bool motor_1_hold

# analog mux and nichrome word
uint8 A
bool burnwire_1_drive
bool burnwire_2_drive

#M1 M2 motors count down until stop driving
uint8 motor_1_secs
uint8 motor_2_secs

#N1 N2 nichrone burn wire statuses
uint16 burnwire_1_secs
uint16 burnwire_2_secs

#TR short deadman (reset by don't abort message)
uint8 short_deadsecs

#TV short idle deadman (usually disabled)
uint8 short_deadsecs_idle

#long deadman
int32 deadsecs

#################
# deadhour_status
int32 Dd