#sub message of Dvl.msg for water current profiling funcionality
#	similar to Ranges3D.msg which is only available in bottom-track
#	Adcp.msg will only be available when in water-track mode

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# ENUM: Coordinates system 
uint8 ADCP_COORD_BEAM=0
uint8 ADCP_COORD_INSTRUMENT=1
uint8 ADCP_COORD_SHIP=2
uint8 ADCP_COORD_EARTH=3


# ENUM: ADCP Type

# Standard array with one piston transducer for each beam
# If you have 4-5 individual disks on your DVL, use this one.
# Examples include Sonardyne Syrinx, Nortek DVLs, and RDI Workhorse
uint8 ADCP_TYPE_PISTON=0

# Newfangled phased-array DVL.  If you have one giant disk with complicated stuff on it,
# probably this one.
# Examples include newer RDI DVLs such as pathfinder, pioneer, and tasman.
uint8 ADCP_TYPE_PHASED_ARRAY=1
#####################################################################################################


# Coordinate system mode (see above)
uint8 coordinate_mode

# Type of DVL array employed (see above).  Necessary when
# deciding how to apply sound velocity corrections
uint8 adcp_type

#num bins 
uint8 cells

#length of each depth bin [m]
uint16 cell_depth

#Distance to middle of first depth bin [m]
float32 bin0_distance

# Raw data (unit vectors of each beam that show geometry of each beam relative to instrument)
geometry_msgs/Vector3[4] beam_unit_vec  # Geometry of each of the 4 beams

#Each element of vel_bin_beams array will correspond to a water 'bin' containing ocean current velocity information depending on coordinate system
WaterCurrentProfile[] vel_bin_beams