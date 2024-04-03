# Just use a simple standard header
std_msgs/Header header

# To ease processing, we need to specify our bounds
float64 northing_max
float64 northing_min

float64 easting_max
float64 easting_min

# + depth is down, so bigger numbers are lower
float64 depth_max
float64 depth_min

# We also need the dimensions of the final map
int32 cells_easting
int32 cells_northing

# and a Holiday (No data) value
float32 nodata_value

# Float32 only has 6-7 digits.  This is only a problem
# when doing millimeter-precision mapping at 1000+m depth,
# but that's a thing that sometimes happens.  We define
# offset fields that let users specify an overall offset.
float64 depth_offset

# Finally, the actual map data as a row-major array
# Note that size(depth_data) == cells_easting * cells_northing
float32[] depth_data
