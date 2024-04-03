# ENUM: Velocity mode
uint8 DVL_MODE_BOTTOM=1
uint8 DVL_MODE_WATER=2

# ENUM: Coordinates system 
uint8 DVL_COORD_BEAM=0
uint8 DVL_COORD_INSTRUMENT=1
uint8 DVL_COORD_SHIP=2
uint8 DVL_COORD_EARTH=3

# ENUM: DVL Type

# Standard array with one piston transducer for each beam
# If you have 4-5 individual disks on your DVL, use this one.
# Examples include Sonardyne Syrinx, Nortek DVLs, and RDI Workhorse
uint8 DVL_TYPE_PISTON=0

# Newfangled phased-array DVL.  If you have one giant disk with complicated stuff on it,
# probably this one.
# Examples include newer RDI DVLs such as pathfinder, pioneer, and tasman.
uint8 DVL_TYPE_PHASED_ARRAY=1

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header
float64 dvl_time

#####################################################################
# Velocities [m/s]
geometry_msgs/Vector3 velocity

# row-major, xyz axes.  Set to "-1" if not known.
float64[9] velocity_covar

float64 altitude    # Average of all good beam ranges: altitude = sum(range) / num_good_beams
float64 course_gnd  # Calculated from xy velocities: course_gnd = atan2(velocity.y, velocity.x)
float64 speed_gnd   # Calculated from xy velocities: speed_gnd = sqrt(velocity.x^2 + velocity.y^2)

uint8 num_good_beams


# Speed of sound (m/s) used by the DVL
float32 speed_sound

#####################################################################
# Raw data
geometry_msgs/Vector3[4] beam_unit_vec  # Geometry of each of the 4 beams

# RDI sends ALTITUDE and calls it RANGE.  We convert back and send actual RANGES, such that
# beam_unit_vec[i] * range[i] is the point where the beam hits the seafloor.  If you want
# the raw uncorrected altitudes, look at the specific datagram message for your DVL (e.g., pd0, pd5, etc).
# If you want altitude, simply multiply by sqrt(3)/2.
float64[4] range
float32[4] range_covar

# A beam quality flag; exactly which flag is used for what is DVL-specific (for now)
float32[4] beam_quality

# Raw velocities [m/s].  Will be beam 1,2,3,4 velocity
# in beam coordinates; or x,y,z,error in other modes
float32[4] raw_velocity
# Covariance, as always, should be -1 if unknown
float32[4] raw_velocity_covar

# Velocity mode (see above)
uint8 velocity_mode

# Coordinate system mode (see above)
uint8 coordinate_mode

# Type of DVL array employed (see above).  Necessary when
# deciding how to apply sound velocity corrections
uint8 dvl_type
