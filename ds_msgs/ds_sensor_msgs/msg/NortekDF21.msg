# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

################################
# Standard DVL type disclaimer #
################################

# Standard array with one piston transducer for each beam
# If you have 4-5 individual disks on your DVL, use this one.
# Examples include Sonardyne Syrinx, Nortek DVLs, and RDI Workhorse
uint8 DVL_TYPE_PISTON=0

# Newfangled phased-array DVL.  If you have one giant disk with complicated stuff on it,
# probably this one.
# Examples include newer RDI DVLs such as pathfinder, pioneer, and tasman.
uint8 DVL_TYPE_PHASED_ARRAY=1

# Type of DVL array employed (see above).  Necessary when
# deciding how to apply sound velocity corrections
uint8 dvl_type

#################
# DVL processed #
#################

float64 altitude_sum
float64 speed_gnd
float64 course_gnd
uint8 good_beams
float64 dvl_time

################
# DF21/22 Mode #
################

#  All distances are in m, velocities in m/s, intensities in dB

uint8 version
uint8 offsetOfData
uint32 serialNumber
uint8 year
uint8 month
uint8 day
uint8 hour
uint8 minute
uint8 seconds
uint16 microSeconds
uint16 nBeams
uint32 error
uint32 status
float32 speed_sound
float32 temperature
float32 pressure

#############
# Beam Data #
#############

float32[4] velBeam    #Velocities for each beam
float32[4] distBeam   #Distances for each beam
float32[4] fomBeam    #Figure of merit for each beam
float32[4] timeDiff1Beam  #DT1 for each beam
float32[4] timeDiff2Beam  #DT2 for each beam
float32[4] timeVelEstBeam #Duration of velocity estimate for each beam

#############
# XYZ Data #
#############
float32 velX  #Velocity X
float32 velY  #Velocity Y
float32 velZ1 #Velocity Z1
float32 velZ2 #Velocity Z2
float32 fomX  #Figure of Merit X
float32 fomY  #Figure of Merit Y
float32 fomZ1 #Figure of Merit Z1
float32 fomZ2 #Figure of Merit Z2
float32 timeDiff1X    #Time from trigger to center of bottom echo
float32 timeDiff1Y    #Same as above
float32 timeDiff1Z1   #Same as above
float32 timeDiff1Z2   #Same as above
float32 timeDiff2X    #Time from start of NMEA output msg to center of bottom echo
float32 timeDiff2Y    #Same as above
float32 timeDiff2Z1   #Same as above
float32 timeDiff2Z2   #Same as above
float32 timeVelEstX   #Duration of velocity estimate for each component
float32 timeVelEstY   #Same as above
float32 timeVelEstZ1  #Same as above
float32 timeVelEstZ2  #Same as above
