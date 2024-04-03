time stamp

# XR Index
int8 idx

# Actual state of things, as of last report
bool[] dcams_actual
bool[] wires_actual

# Currently-commanded state
bool[] dcams_cmds
bool[] wires_cmds

# Countdown timers
uint8 XR_COUNTDOWN_DISABLE=0
uint8 XR_COUNTDOWN_EXPIRED=1
uint8 dcam_countdown_sec
uint8 wire_countdown_sec
uint32 deadman_sec

