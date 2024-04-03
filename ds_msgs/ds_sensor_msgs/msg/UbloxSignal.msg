

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

# Signal ID (each satellite can broadcast multiple signals)
string sig_id

# Carrier-to-noise-density ratio (dBHz)
float32 carrier_noise

# Pseudorange residual, in meters
float32 pseudorange_residual

# Signal quality
uint16 quality_indicator
uint16 QUALITY_NOSIGNAL = 0
uint16 QUALITY_SEARCHING = 1
uint16 QUALITY_ACQUIRED = 2
uint16 QUALITY_UNUSABLE = 3
uint16 QUALITY_CODELOCKED = 4
uint16 QUALITY_CODECARRIERTIME_LOCKED_5 = 5
uint16 QUALITY_CODECARRIERTIME_LOCKED_6 = 6
uint16 QUALITY_CODECARRIERTIME_LOCKED_7 = 7

# Correction Source
uint16 correction_source
uint16 CORRECTIONS_NONE = 0
uint16 CORRECTIONS_SBAS = 1
uint16 CORRECTIONS_BEIDOU = 2
uint16 CORRECTIONS_RTCM2 = 3
uint16 CORRECTIONS_RTCM3OSR = 4
uint16 CORRECTIONS_RTCM3SSR = 5
uint16 CORRECTIONS_QZSSSLAS = 6

# Ionosphere Model Used
uint16 ionosphere_model
uint16 IONOMODEL_NONE = 0
uint16 IONOMODEL_KLOBUCHARGPS = 1
uint16 IONOMODEL_SBAS = 2
uint16 IONOMODEL_KLOBUCHARBEIDOU = 3

# flags
uint16 health
uint16 HEALTH_UNKNOWN = 0
uint16 HEALTH_HEALTHY = 1
uint16 HEALTH_UNHEALTH = 2

bool pseudorange_smoothed
bool pseudorange_used
bool carrier_range_used
bool doppler_used
bool pseudorange_corrections_applied
bool carrier_range_corrections_applied
bool doppler_corrections_applied
