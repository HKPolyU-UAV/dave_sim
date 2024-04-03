# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

################################
# Standard DVL type disclaimer #
################################

# Standard array with one piston transducer for each beam
# If you have 4-5 individual disks on your DVL, use this one.
# Examples include Sonardyne Syrinx, Nortek DVLs, and RDI Workhorse
uint8 DVL_TYPE_PISTON=0

# Newfangled phased-array DVL.  If you have one giant disk with complicated stuff on it,
# probably this one.
# Examples include newer RDI DVLs such as pathfinder, pioneer, and tasman.
uint8 DVL_TYPE_PHASED_ARRAY=1

# Type of DVL array employed (see above).  Necessary when
# deciding how to apply sound velocity corrections
uint8 dvl_type

#################
# DVL processed #
#################

float64 altitude_sum
float64 speed_gnd
float64 course_gnd
uint8 good_beams
float64 dvl_time

#################
# Raw Data      #
#################

uint8 config
uint8 coord_mode
float64[4] velocity
float64[4] range
uint8 bottom_status
float64[4] layer_velocity
float64 ref_layer_start
float64 ref_layer_end
uint8 ref_layer_status
uint8 tofp_hour
uint8 tofp_minute
uint8 tofp_second
uint8 tofp_hundreths
uint16 bit_results
uint32 sound_vel
float64 temperature
float64 salinity
float64 depth
float64 pitch
float64 roll
float64 heading
uint32 bt_distance_east
uint32 bt_distance_north
uint32 bt_distance_up
uint32 bt_distance_error
uint32 ref_distance_east
uint32 ref_distance_north
uint32 ref_distance_up
uint32 ref_distance_error

