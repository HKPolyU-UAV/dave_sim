# ZDA Time of day
#            1     2 3 4 5 6 7
#            |     | | | | | |
# $--ZDA,hhmmss.ss,x,x,x,x,x*hh
# 1). UTC Time (hhmmss.ss format)
# 2). Day (01-31)
# 3). Month (01-12)
# 4). Year (4-digit)
# 5). Local zone description, 00 to +/- 13 hours
# 6). Local zone minutes description (same sign as hours)
# 7). Checksum

string talker
time utc_time
duration zone_offset
uint8 checksum
