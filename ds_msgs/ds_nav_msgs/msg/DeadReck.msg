# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# Raw dvl velocities in instrument frame
geometry_msgs/Vector3 dvl_speed
# Raw dvl velocities in instrument frame, scaled by speed of sound ratio
geometry_msgs/Vector3 dvl_scaled
# Dvl velocities in vehicle frame, scaled by speed of sound ratio
geometry_msgs/Vector3 dvl_scaled_tfd
# Dvl velocities in world frame, scaled by speed of sound ratio
geometry_msgs/Vector3 dvl_scaled_tfd_world


# Integration timestep
float64 dt

# Ratio between used sound speed and dvl sound speed
float64 scale_factor
float64 used_ss
float64 dvl_ss

# Define source of velocities being used
uint8 SOURCE_DVL=0
uint8 SOURCE_DVL_HOLD=1
uint8 SOURCE_MODEL=2
uint8 active_velocity_source