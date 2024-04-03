# Specialized message type for sending/receiving micromodem data
# All other modems should use AcousticModemData

time stamp
uint16 local_addr
uint16 remote_addr

# Rate for the micromodem; we typically use 1, 4 or 5
uint8 rate

ModemData[] frames
