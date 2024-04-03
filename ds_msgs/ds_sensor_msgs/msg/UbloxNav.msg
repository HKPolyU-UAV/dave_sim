# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# Mostly covered by the PVT messages, but can be overwritten by the high-precision
# Latlon position message if available

# Position
float64 latitude
float64 longitude
float64 height_ellipsoid
float64 height_msl

# Velocity
float32 velocity_north # m/s
float32 velocity_east  # m/s
float32 velocity_down  # m/s

# Accuracy Stats
float32 accuracy_horz # m
float32 accuracy_vert # m
float32 accuracy_time # s
float32 accuracy_vel  # m/s

# Status
uint64 time_of_week # used to aggregate messages
string fixtype

# Relative position stuff
uint16 relpos_station_id
float64 relpos_north # m
float64 relpos_east # m
float64 relpos_down # m
float64 relpos_length # m
float64 relpos_heading # deg

float32 relpos_accuracy_north
float32 relpos_accuracy_east
float32 relpos_accuracy_down
float32 relpos_accuracy_length
float32 relpos_accuracy_heading

uint16 relpos_flags
uint16 RELPOS_FLAGS_GNSSOK = 1 # 0x01
uint16 RELPOS_FLAGS_DIFFSOLN = 2 # 0x02
uint16 RELPOS_FLAGS_RELPOSVALID = 4 #0x04
uint16 RELPOS_FLAGS_CARRSOLN_MASK = 24 #0x18
uint16 RELPOS_FLAGS_ISMOVING = 32 # 0x20
uint16 RELPOS_FLAGS_REFPOSMISS = 64 # 0x40
uint16 RELPOS_FLAGS_REFOBSMISS = 128 # 0x80


