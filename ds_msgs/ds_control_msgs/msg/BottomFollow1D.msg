# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

float64 depth_bot
float64[2] depth_env
float64 depth_goal
float64 ref_speed
float64 depth_rate_filtered
float64[5] past_altitude_values
float64 median_altitude
float64 commanded_altitude
float64 env_padding_speed
float64 env_padding_accel
float64 step
float64 last_good_altitude
float64 last_good_depth
float64 last_good_depth_bot
float64 depth_floor
float64 alarm_timeout
float64 min_speed
float64 speed_gain
duration time_inside_env
duration alt_bad_timer

time last_good_altitude_time
time time_low_depth_rate
time last_altitude_time

bool alarm
bool stuck

int8 pseudo_bot_code

# copy of the smoothing parameters to sync the reference smoother
float64 depth_rate_d
float64 depth_accel_d

uint64 bad_altitude_hits

# The raw altitude after tf. This can be negative if bad, for mc descent
float64 raw_altitude

# User push-up or push-down override
# When the override begins
time userDepthOverrideEngageTime
# Whether or not the override is engaged for the current control loop
bool userDepthOverrideEngaged
# When the override ends
time userDepthOverrideEndTime
# Override direction
int8 override_depth_direction
