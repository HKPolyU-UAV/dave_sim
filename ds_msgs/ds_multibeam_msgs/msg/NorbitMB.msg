# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# Per Ping Info
float32 snd_velocity
float32 sample_rate
uint32 num_beams
uint32 ping_num
uint8 percent_good
uint16 max_depth
uint16 min_depth
uint16 center_depth
uint16 max_range
uint16 min_range
uint16 center_range
float64 ping_time
float64 time_net
float32 ping_rate
uint16 bathy_datatype
uint16[8] beam_dist_mode
uint8 sonar_mode
float32 tx_angle
float32 gain
float32 tx_freq
float32 tx_bw
float32 tx_len
float32 tx_voltage
float32 swath_dir
float32 swath_open
float32 gate_tilt

# Per Beam Information
uint32[] sample_num
float32[] beam_distance
float32[] angle
uint16[] upper_gate
uint16[] lower_gate
float32[] intensity
uint16[] flags
uint16[] quality_flag
uint16[] quality_val

