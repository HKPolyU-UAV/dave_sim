float64 OXYGEN_CONCENTRATION_NO_DATA=-9999.00

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# O2 concentration in uM
float64 concentration
float64 concentration_covar

# Saturation in %
float64 air_saturation
float64 air_saturation_covar

# Temperature in C
float64 temperature
float64 temperature_covar
