float32 POWERSUPPLY_NODATA = -9999.99

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# ID number to help identify chargers
int32 idnum

# Measured output values
float32 meas_volts
float32 meas_amps

# Programmed output values
float32 prog_volts
float32 prog_amps

# Flag for whether the output is enabled or not
bool output_enable

# True if the output is good; false otherwise
bool status_good

# If true, the front panel is locked out and local control is disabled
bool frontpanel_locked

# Independent status flags.  These SHOULD be mutually exclusive,
# but the Lambda power supplie provide one for each
bool fault
bool constant_voltage
bool constant_current

# It is 
string status_msg


