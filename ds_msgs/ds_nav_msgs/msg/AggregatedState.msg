# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# Position
FlaggedDouble northing
FlaggedDouble easting
FlaggedDouble down
FlaggedDouble roll
FlaggedDouble pitch
FlaggedDouble heading

# Rates
FlaggedDouble surge_u
FlaggedDouble sway_v
FlaggedDouble heave_w
FlaggedDouble p
FlaggedDouble q
FlaggedDouble r

# Accelerations
FlaggedDouble du_dt
FlaggedDouble dv_dt
FlaggedDouble dw_dt
FlaggedDouble dp_dt
FlaggedDouble dq_dt
FlaggedDouble dr_dt
