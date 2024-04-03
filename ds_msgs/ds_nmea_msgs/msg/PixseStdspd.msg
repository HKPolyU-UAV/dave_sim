# $PIXSE,STDSPD,x.xxx,y.yyy,z.zzz*hh<CR><LF>
# where:
# x.xxx is the east speed std dev in the navigation reference frame (m/s)
# y.yyy is the north speed std dev in the navigation reference frame (m/s)
# z.zzz is the vertical speed std dev in the navigation reference frame (m/s)
# hh is the checksum

float64 east
float64 north
float64 vertical
uint8 checksum
