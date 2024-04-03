float64 CTD_NO_DATA=-9999.0

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# Conductivity in S/m
float64 conductivity
float64 conductivity_covar

# Temperature in ITS-90 deg C
float64 temperature
float64 temperature_covar

# Pressure, in Decibars
# Pressure-Depth conversions using Fofonoff are emitted using
# DepthPressure.msg messages
float64 pressure
float64 pressure_covar

# Salinity, in psu
float64 salinity
float64 salinity_covar

# Speed of Sound
float64 sound_speed
float64 sound_speed_covar

