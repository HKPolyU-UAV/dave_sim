# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

HTP htp

# Ground fault, from 0 to 100%.  Almost-certainly useless for Sentry.
# If wired properly, the manual has this little table:
# Ground Fault Bus : reading
# None:   0%
# GND :  27%
#  5V :  39%
# 12V :  55%
# 32V : 100%
float32 ground_fault

bool probe_fail
bool leak