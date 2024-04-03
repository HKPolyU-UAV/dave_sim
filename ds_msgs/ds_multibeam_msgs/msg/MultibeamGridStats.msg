# Total soundings
uint64 soundings_total

# Added to grid
uint64 soundings_accepted

# Soundings rejected because they were flagged
uint64 soundings_flagged

# Soundings dropped by the gridder
uint64 soundings_dropped

# Grid dimensions
float32 cell_size

# Grid dimensions in cells
uint32 width
uint32 height

# Grid cells filled in
uint64 grid_cells_used
uint64 grid_cells_total

# Grid memory usage stats
float32 grid_memory_mb
float32 grid_memory_limit_mb

# Grid location
float64 grid_origin_easting
float64 grid_origin_northing
