# $PIXSE,HSPOS_,hhmmss.ss,llmm.mmmmm,H,LLmm.mmmmm,D,d.dd,a.ad,x.xx,y.yy,z.zz,d.dd,nn,
#          c,e.e,n.n,m.mmmm,s.ssss,vvvv.v<CR><LF>
# where:
# hhmmss.ss      is the validity time of the latitude/longitude data transmitted in the
#                UTC time reference frame if available otherwise in the system
#                time reference frame
# llmm.mmmmm     is the latitude in deg, minutes, decimal minutes
# H              N: north, S: south
# LLLmm.mmmmm    is the longitude in deg, minutes, decimal minutes
# D              E: east, W: west
# d.dd           is the depth in meters
# a.aa           is the altitude in meters (from DVL)
# x.xx           is the latitude Std (meters)
# y.yy           is the longitude Std (meters)
# z.zz           is the latitude longitude error covariance (meters)
# d.dd           is the depth Std (meters)
# nn             is the longitude UTM zone (integer)
# c              is the latitude UTM zone (character)
# e.e            is the East UTM position (meters)
# n.n            is the North UTM position (meters)
# m.mmmm         is the log misalignment according to the heading in degrees
# s.ssss         is the log scale factor error estimation in %
# vvvv.v         is the sound velocity in m/s

time             timestamp
float64          latitude
uint8            latitude_dir
float64          longitude
uint8            longitude_dir
float64          depth
float64          altitude
float64          latitude_stdev
float64          longitude_stdev
float64          depth_stdev
int16            longitude_utm_zone
uint8            latitude_utm_zone
float64          eastings
float64          northings
float64          heading_misalignment
float64          heading_misalignment_scale_factor
float64          sound_velocity