# A string identifying which GNSS system is in use (e.g., GPS, BeiDou, GLONASS, etc)
string gnss_id

# The ID of the specific satellite that broadcast the signal.  Includes
# a letter indicating the service.  Example: GPS satellite #4 would be G4.
# IDs:
# G: GPS (US system)
# S: SBAS (Satellite Based Augmentation System, so WAAS, EGNOS, SDCM, GAGAN, etc)
# E: Galileo (European system)
# B: BeiDou (Chinese system)
# Q: QZSS (Japanese SBAS that also transmits GPS-style timing signals)
# R: GLONASS (Russian system)
string sat_id

# Carrier-to-noise-density ratio (dBHz)
float32 carrier_noise

# Pseudorange residual, in meters
float32 pseudorange_residual

# Satellite location, in degrees
float32 elevation
float32 azimuth

# Quality indicator
uint8 quality_indicator
uint8 QUALITY_NOSIGNAL = 0
uint8 QUALITY_SEARCHING = 1
uint8 QUALITY_ACQUIRED = 2
uint8 QUALITY_UNUSABLE = 3
uint8 QUALITY_CODELOCKED = 4
uint8 QUALITY_CODECARRIERTIME_LOCKED_5 = 5
uint8 QUALITY_CODECARRIERTIME_LOCKED_6 = 6
uint8 QUALITY_CODECARRIERTIME_LOCKED_7 = 7

# flags
uint8 health
uint8 HEALTH_UNKNOWN = 0
uint8 HEALTH_HEALTHY = 1
uint8 HEALTH_UNHEALTH = 2

bool differential_available
bool pseudorange_smoothed

uint8 oribt_source
uint8 ORBIT_SOURCE_NONE = 0
uint8 ORBIT_SOURCE_EPHEMERIS = 1
uint8 ORBIT_SOURCE_ALMANAC = 2
uint8 ORBIT_SOURCE_ASSISTNOW_OFFLINE = 3
uint8 ORBIT_SOURCE_ASSISTNOW_AUTONOMOUS = 4
uint8 ORBIT_SOURCE_OTHER5 = 5
uint8 ORBIT_SOURCE_OTHER6 = 6
uint8 ORBIT_SOURCE_OTHER7 = 7

bool ephAvail
bool almAvail
bool anoAvail
bool aopAvail
bool sbasCorrUsed
bool rtcmCorrUsed
bool slasCorrUsed
bool pseudorange_corrections_applied
bool carrier_range_corrections_applied
bool doppler_corrections_applied