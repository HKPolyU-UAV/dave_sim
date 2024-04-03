# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# GPS time of this message
uint32 time_of_week

# So you probably want to call this one "signals".  But don't do that.  QT won't like it.  A LOT.
UbloxSignal[] sat_signals