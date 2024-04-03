# .       1      2 3        4 5         6 7
#         |      | |        | |         | |
# $--GLL,llll.ll,a,yyyyy.yy,a,hhmmss.ss,A*hh
# 1) Latitude
# 2) N or S (North or South)
# 3) Longitude
# 4) E or W (East or West)
# 5) Time (UTC)
# 6) Status A - Data Valid, V - Data Invalid
# 7) Checksum

float64    GLL_NO_DATA=-999.999

uint8 GLL_DATA_VALID=65     # (ascii 'A')
uint8 GLL_DATA_INVALID=86   # (ascii 'V')

string     talker
float64    latitude
uint8      latitude_dir
float64    longitude
uint8      longitude_dir
time       timestamp
uint8      status
uint8      checksum