# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# Watercolumn data
float32 snd_velocity
float32 sample_rate
uint32 num_beams
uint32 sample_num
float32 ping_time
uint32 dtype
int32 t0
float32 gain
float32 swath_dir
float32 swath_open
float32 tx_freq
float32 tx_bw
float32 tx_len
float32 tx_amp
float32 ping_rate
uint32 ping_num
float32 time_net
uint32 beams
int32 vga_t1
float32 vga_g1
int32 vga_t2
float32 vga_g2
float32 tx_angle
float32 tx_voltage
uint16[8] beam_dist_mode
uint8 sonar_mode
float32 gate_tilt

# Pixel msg
uint32[] pixel_data
float32[] beam_dir
float32[] beam_distance


