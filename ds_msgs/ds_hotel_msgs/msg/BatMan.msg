# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

int32 num_bats

float32 maxModuleVolt
float32 minModuleVolt
float32 maxSwitchTemp
float32 minSwitchTemp

float32 maxCellVolt
float32 minCellVolt
float32 maxCellTemp
float32 minCellTemp

float64 capacityCoulombs
float64 capacityAh
float64 chargeCoulombs
float64 chargeAh
float64 percentFull

float32[] moduleVolt
float64[] moduleAh
float64[] moduleCapacity
float64[] modulePercent
bool[] moduleDischarging
bool[] moduleCharging
