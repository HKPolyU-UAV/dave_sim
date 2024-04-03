# Message for a Gyro
float64 GYRO_NO_DATA = -9999.0

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# In degrees.  Standard "Forward - Up - Left" coorinates
float64 roll               # + rolls to right
float64 pitch              # + pitch nose down
float64 heading            # + turns to left

float64 roll_covar
float64 pitch_covar
float64 heading_covar

geometry_msgs/Quaternion orientation
