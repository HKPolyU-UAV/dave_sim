# This type is a general payload command for use by the Imaging Payload series of Tasks

# This message includes a command to execute and a set of key/value configuration pairs.

# This value indicates that only the config should be updated
uint16 COMMAND_CONFIGONLY=0

# Start imaging
uint16 COMMAND_START=1

# Stop imaging.  Has the same effect as "powerup"
uint16 COMMAND_STOP=2

# Power up the instrument.  Has the same effect as "stop"
uint16 COMMAND_POWERUP=3

# Tells the PayloadManager it should stop the payload and shut it down of noone asks for it in a few minutes
uint16 COMMAND_SHUTDOWN=4

# Tells the PayloadManager
uint16 COMMAND_NEWLINE=5

uint16 command

time stamp

# We also include a dictionary of key/value pairs passed directly from the mission
ds_core_msgs/KeyString[] config
