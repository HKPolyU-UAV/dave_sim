# VTG Track Made Good and Ground Speed
#        1   2 3   4 5   6 7   8 9
#        |   | |   | |   | |   | |
# $--VTG,x.x,T,x.x,M,x.x,N,x.x,K,N*hh
# 1) Track Degrees
# 2) T = True
# 3) Track Degrees
# 4) M = Magnetic
# 5) Speed Knots
# 6) N = Knots
# 7) Speed Kph
# 8) K = Kilometres Per Hour
# 9) Mode Indicator
#    A = Automoous
#    D = Differential
#    E = Estimated / Dead Reckoning
#    M = Manual input
#    N = Data not valid.
# 9) Checksum

float64    VTG_NO_DATA=-999.999

string     talker
float64    track_degrees_true
float64    track_degrees_magnetic
float64    speed_knots
float64    speed_km_per_hour
string     mode
uint8      checksum
