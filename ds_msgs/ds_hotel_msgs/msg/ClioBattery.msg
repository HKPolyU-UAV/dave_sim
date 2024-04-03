# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

uint16 busAddress
uint16 version
string date
uint16 startupCondition
uint16 uptime
uint16 overflowCount
uint16 timeoutCount
uint16 powerswitchState
uint16 chargetimeRemaining
uint16 chargetimeMax
uint16 moduleStatus # Bitwise OR of status fields from all batteries

ClioBatteryPack[] packs

## Calculated
float32 maxPackTemp
float32 minPackTemp
float32 maxPackVoltage
float32 minPackVoltage
float32 minCellVoltage
float32 maxCellVoltage
float32 moduleVoltage #V
bool chargeEnabled
bool charging
bool discharging
float32 remainingCapacity # Remaining capacity, in Amp Hours
float32 fullCapacity # Predicted capacity when full, in Amp Hours
float32 designCapacity # Design capacity, in Amp Hours
float64 percentFull # Battery capacity as a percentage.  Local capacity
