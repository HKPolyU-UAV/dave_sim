# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# Attiutde in DEGREES
float64 heading_deg
float64 pitch_deg
float64 roll_deg

# Position in DEGREES
float64 latitude_deg
float64 longitude_deg

# Altitude in meters
float64 altitude_m

# Speed in meters/second
float64 east_speed_m_s
float64 north_speed_m_s
float64 up_speed_m_s

# UTM zone information
uint8 lat_utm_zone           # This is a char
uint8 lon_utm_zone
float64 utm_east_pos_m
float64 utm_north_pos_m
float64 utm_altitude_m

float64 surge_m
float64 sway_m
float64 heave_m

float64 lat_std_dev_m
float64 lon_std_dev_m
float64 altitude_std_dev_m

float64 east_speed_std_dev_m_s
float64 north_speed_std_dev_m_s
float64 up_speed_std_dev_m_s

float64 time_pos

string algorithm_status_lsb
string algorithm_status_msb

string system_status_lsb
string system_status_msb

string user_status