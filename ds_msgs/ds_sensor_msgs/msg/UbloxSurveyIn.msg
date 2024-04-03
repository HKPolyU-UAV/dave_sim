# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# GPS time of this message
uint32 time_of_week

# Surveyed-in position, ecef
geometry_msgs/Point mean_ecef

# Accuracy of the survey-in, in meters
float32 surveyin_accuracy

# Survey-in observation time
uint32 duration

# Number of observations used during this survey-in
uint32 observations

# True when the surveyed-in position is valid
bool valid

# True when the GPS is currently surveying itself in
bool active