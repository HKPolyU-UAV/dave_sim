# $PIXSE,SPEED_,x.xxx,y.yyy,z.zzz*hh<CR><LF>
# where:
# x.xxx is the east speed in m/s (in terrestrial reference frame)
# y.yyy is the north speed in m/s (in terrestrial reference frame)
# z.zzz is the vertical (Sign “+” towards up side) speed in m/s (in terrestrial reference frame)
# hh is the checksum

float64 east
float64 north
float64 vertical
uint8 checksum