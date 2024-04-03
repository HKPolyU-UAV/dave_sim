# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

uint16 link_state
string timestamp_date
string timestamp_time
uint16 tdma_mode
float32 carrier_rate
float32 tx_rate
float32 rx_rate
float32 fec_corrects
float32 fec_failure
float32 deadlink_timer
float32 checksum_failures
float32 tx_kbps
float32 rx_kbps 
float32 corr_val 
float32 hv_dac 
float32 dc_adc 
float32 ac_adc 
float32 backscatter_adc 
float32 gating_mode
float32 read_reg_2
float32 read_reg_3
float32 read_reg_4
float32 housing_humidity
float32 housing_temp
float32 env_err
float32 mode_indicator
float32 system_voltage
