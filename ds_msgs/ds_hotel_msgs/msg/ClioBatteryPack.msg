# Per-pack
float32 temperature # deg C
float32 voltage # V
float32 current
float32 avgCurrent
uint16 maxError
float32 relativeSOC # percent
float32 absoluteSOC # percent
float32 remainingCapacity # Remaining capacity in Amp*hours
float32 fullCapacity # Predicted capacity when fully charged in Amp*hours
uint16 status
float32 countCycle
float32 capacityDesign
string dateManufacture
uint16 serialNumber
# high / mid / low refer to the arrangement of cells within 
# the pack, NOT to which has the highest voltage
float32 highCellVoltage
float32 mid1CellVoltage
float32 mid2CellVoltage
float32 lowCellVoltage

