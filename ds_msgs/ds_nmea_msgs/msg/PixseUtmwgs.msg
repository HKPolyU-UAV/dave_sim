# $PIXSE,UTMWGS,c,nn,x.xxx,y.yyy,z.zzz*hh<CR><LF>
# where:
# C is the latitude UTM zone (character)
# nn is the longitude UTM zone (integer)
# x.xxx is the east UTM position in meter
# y.yyy is the north UTM position in meter
# z.zzz is the altitude in meters
# hh is the checksum

uint8 latitude_utm_zone
int16 longitude_utm_zone
float64 eastings
float64 northings
float64 altitude
uint8 checksum
