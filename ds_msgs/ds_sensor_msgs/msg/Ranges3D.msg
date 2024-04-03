# Includes a variable sized vector of ranges, that a sensor could output
# if it can provide multiple ranges per message (i.e. dvl, multibeam, blueview, ...)
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

Range3D[] ranges

# Corrected range = (actual soundspeed)/(1500)*(original range)
uint8 SOUNDSPEED_CORRECTION_NORMAL=0

# Look this up in the manual
uint8 SOUNDSPEED_CORRECTION_PHASEDARRAYDVL=1

uint8 soundspeed_correction_type
