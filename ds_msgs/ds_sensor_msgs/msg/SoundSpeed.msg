float32 NO_SOUND_SPEED_DATA=-9999.0

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# Speed in m/s
float64 raw_speed
float64 speed
float64 speed_covar

# Enum to represent the source of a sound velocity measurement
# If you have a choice of method, Chen-Millero is probably a good
# bet for anything CTD-cast related.

# Unknown.  Don't use this.
uint8 SV_METHOD_UNKNOWN=0

# Direct measurement.  Usually via a Sound Velocity Probe that shoots actual sound
uint8 SV_METHOD_DIRECT=1

# Chen-Millero algorithm best for depth < 1000m
uint8 SV_METHOD_CHEN_MILLERO=2

# The Del Grosso algorithm best for depth > 1000m
uint8 SV_METHOD_DEL_GROSSO=3

# User defined method.
uint8 SV_METHOD_USER_DEFINED=4

uint8 speed_method

uint8 SV_STATUS_NOT_CLAMPED=0
uint8 SV_STATUS_CLAMPED=1

uint8 clamping_status