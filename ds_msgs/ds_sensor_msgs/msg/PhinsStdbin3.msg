# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# This includes as much data from the Phins STDBIN Version 3 binary message.
# See the document 7-INS-InterfaceLibrary_MU-INSIII-AN-001-U.pdf for details on this
# format.  Where possible, the coordinate system for each field has been included in the
# field.  XV(1,2,3) are the body-frame.  XV(1,2,3)H are a body-centered local/level frame.
# East, North, Up, and Down are geographic.  Where possible, units are
# meters, meters/sec, degrees, degrees per second, etc.

# We include a couple header fields so at least w know what might be valid
uint32 nav_fields
uint32 extended_nav_fields
uint32 external_data_fields
float64 nav_validity_time
uint32 counter

# Heading, pitch, and roll have their own definitions
float32 heading
float32 roll
float32 pitch
float32 heading_stddev
float32 roll_stddev
float32 pitch_stddev

# Real-time heave at the selected lever-arm, in meters
float32[3] rt_heave_XVnH

# Time of validity for smart heave, in seconds
float64 smart_heave_validity_time

# Smart heave, at primary lever-arm.  Meters, positive up, XV3H
float32 smart_heave

# Heading, pitch, and roll rate.  Degrees / s
float32 heading_rate
float32 roll_rate
float32 pitch_rate

# Body rotation rates, degrees/s
float32[3] body_rates_XVn

# Body-frame accelerations at the primary lever-arm, m/s^2
# Corrected for gravity and Coriolois
float32[3] body_accel_XVn

# Lat/lon in degrees
float64 latitude
float64 longitude

# Altitude in meters, positive up.  Referenced as shown in reference field
float32 altitude
uint8 altitude_reference
uint8 ALTREF_GEOID = 0
uint8 ALTREF_ELLPS = 1

# Position covariance matrix, meters, row-major (doesnt' matter really...)
# north first
float32[4] position_cov
float32 altitude_stddev

# Geographic velocity, m/s, north/east/up
float32[3] velocity_NEU
float32[3] velocity_stddev_NEU

# Geographic current estimate, m/s, north/east
float32[2] current_NE
float32[2] current_stddev_NE

uint8 day
uint8 month
uint16 year

# Status words are recorded, but not decoded.  Refer to the manual for details
uint32[2] sensor_status
uint32[4] ins_algo_status
uint32[3] ins_system_status
uint32 ins_user_status
uint32 ahrs_algo_status
uint32[3] ahrs_system_status
uint32 ahrs_user_status

# m/s.  Note we put in the order 1-2-3, NOT the XV3H order shown in the manual
float32[3] heaveSurgeSway_speed_XVnH

# Body-frame velocity, primary lever-arm
float32[3] body_velocity_XVn

# Geographic frame accelerations, primary lever-arm.  m/s^2.  Not compensated for gravity.
# North East Up
float32[3] geo_accel_NEU

# degrees
float32 course_over_ground

# m/s
float32 speed_over_ground

# Average FOG temperature, deg C
float32 temp_avg_fog

# Average Accelerometer temperature, deg C
float32 temp_avg_acc

# Sensor Board temperature, deg C
float32 temp_board

# The attitude quaternion may or may not conform to the ROS standard
# Until that's determined, it stays a float32[4]
float32[4] attitude_quaternion
float32[3] attitude_quaternion_stddev

# Raw body-frame accelerations, m/s^2.  VERY quantized on non-export-controlled Phins.
float32[3] raw_accel_XVn

# Body-frame acceleration stddev, m/s^2
float32[3] body_accel_stddev_XVn

# Body-frame rotation rate standard deviation
float32[3] body_rotrate_stddev_XVn

# Raw body-rate rotation acceleration
float32[3] raw_rot_acc_XVn
float32[3] raw_rot_acc_stddev_XVn

# Raw vessel-frame rotation rate, not compensated for earth rate.  Quantized on
# non-export-controlled Phins
float32[3] raw_rot_rate_XVn


