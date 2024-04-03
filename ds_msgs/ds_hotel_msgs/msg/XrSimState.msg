time stamp

# XR Index
int8 idx

# Actual state of things, as of last report
bool[] dcams_actual
bool[] wires_actual

# Currently-commanded state
bool[] dcams_cmds
bool[] wires_cmds

# Seconds burnwire, etc are to remain active
float32[] motor_secs
float32[] burnwire_secs

# Short deadman
float32 short_deadsecs

# Long deadman
float32 deadsecs

# This is an acoustic command code that gets passed along to the status message
uint8 acoustic_code

# How open a dcam is (from 0% to 100%)
float32[] dcam_open_pct

# Values for burned a wire is.  From "100" for fully intact to "0" 
# for fully burned
float32[] wire_burn_pct
