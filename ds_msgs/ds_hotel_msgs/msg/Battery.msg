# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

int32 idnum

float32[] voltages      #V
uint16[] temperatures   #deg C

float32 maxCellVoltage
float32 minCellVoltage
uint16 maxCellTemp
uint16 minCellTemp

float32 totalVoltage     # V
uint16 switchTemp       # deg C
uint32 countFull        # Raw counts
uint32 countCapacity    # countCapacity = countUsed + countAvailable
uint32 countFreerun
int32 countUsed
int32 countAvailable
bool balanceEnable
float64 balanceGain
float64 balanceOnsetVoltage
float64 balanceMaxCurrent
uint64 balanceCommandId

# Raw Batttery status info
bool charging
bool discharging
int32 safetyCode
int32 problemLocation
int32 numScans
int32 overflowCount
int32 timeoutCount
int32 timeoutLocation
int32 uptime
int32 fixCount
string safetyString
string interruptProblemLocation

# Calculated status info
uint16 errorStatus
float64 capacityCoulombs  # Capacity of this battery, in Amp*hours
float64 capacityAh
float64 chargeCoulombs    # Charge currently in the battery, in Amp*hours
float64 chargeAh
float64 percentFull       # Battery capacity as a percentage

uint64 repeatingCommandId