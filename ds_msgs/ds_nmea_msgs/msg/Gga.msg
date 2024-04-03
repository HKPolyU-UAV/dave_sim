#         1         2      3 4        5 6 7  8   9   10 |  12 13  14   15
#         |         |      | |        | | |  |   |   |  |  |  |   |    |
# $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
#  1) Time (UTC)
#  2) Latitude
#  3) N or S (North or South)
#  4) Longitude
#  5) E or W (East or West)
#  6) GPS Quality Indicator,
#  0 - fix not available,
#  1 - GPS fix,
#  2 - Differential GPS fix
#  7) Number of satellites in view, 00 - 12
#  8) Horizontal Dilution of precision
#  9) Antenna Altitude above/below mean-sea-level (geoid)
# 10) Units of antenna altitude, meters
# 11) Geoidal separation, the difference between the WGS-84 earth ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid
# 12) Units of geoidal separation, meters
# 13) Age of differential GPS data, time in seconds since last SC104 type 1 or 9 update, null field when DGPS is not used
# 14) Differential reference station ID, 0000-1023
# 15) Checksum

# No data value
float64 GGA_NO_DATA=-999.9999

# Enum values for fix_quality field.
uint8 FIX_NONE=0
uint8 FIX_GPS=1
uint8 FIX_DGPS=2

string           talker
time             timestamp
float64          latitude
uint8            latitude_dir
float64          longitude
uint8            longitude_dir
uint8            fix_quality
uint8            num_satellites
float64          hdop
float64          antenna_alt
uint8            antenna_alt_unit
float64          geoid_separation
uint8            geoid_separation_unit
float64          dgps_age
uint16           dgps_ref
uint8            checksum


