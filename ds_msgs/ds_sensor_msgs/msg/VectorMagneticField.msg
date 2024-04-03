float64 NO_VECTOR_MAGNETIC_FIELD_DATA=-9999.0

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# All magnetic field values are in nT
float64 x
float64 y
float64 z

# Covariance matrix for magnetic field measurements in row-major order
float64[9] field_covar

# Temperature in C (usually nearby the magnetometer sensor, inside housing)
float64 temperature
float64 temperature_covar
