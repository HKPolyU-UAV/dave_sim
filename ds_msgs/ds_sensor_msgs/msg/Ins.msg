# Message for an Inertial Navigation System 
# operating not-quite in INS mode

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# In degrees.  For humans.
# Follows Fossen N-E-D notation
float64 roll
float64 pitch
float64 heading

# modelled after sensor_msgs/Imu.msg

# Global-frame, E/N/U
geometry_msgs/Quaternion orientation

# Global-frame, X/Y/Z
geometry_msgs/Vector3 angular_velocity

# Global-frame, E-N-U
geometry_msgs/Vector3 linear_velocity

# Global-frame, E-N-U
geometry_msgs/Vector3 linear_acceleration


# VERY approximate latitude
float64 latitude


