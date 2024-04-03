# This message was originally written for a Lord Microstrain 3DM-GX5-25, but
# can be used for any high-rate 9-axis IMU that does its own integration and reports deltas

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

geometry_msgs/Quaternion orientation

# Angular velocity, in radians/s
geometry_msgs/Vector3 angular_velocity

# Integrated gyros, in radians
geometry_msgs/Vector3 angular_delta

# Linear acceleration, in m/s^2
geometry_msgs/Vector3 linear_acceleration

# Integrated accelerometers, in m/s
geometry_msgs/Vector3 linear_delta

# Magnetometer, in Gauss
geometry_msgs/Vector3 magnetometer

# GPS raw timestamp
uint16 gps_week
float64 gps_time_of_week


