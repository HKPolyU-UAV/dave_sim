#
# Acoustic Modem Data
#
# Message type for sending/receiving data via acoustic modems

# Timestamp
time stamp

# Local modem address
#
# This is the address of the modem that received the data
uint16 local_addr

# Remote modem address
#
# This is the address of the modem that sent the data
uint16 remote_addr

# Data payload
uint8[] payload