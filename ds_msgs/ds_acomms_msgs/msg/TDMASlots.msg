# Starting time for each slot, in seconds after start of the minute
float32[] starts
# Duration of each slot
float32[] durations
# rate for each slot
int8[] rates

uint8 DEFAULT=0 # Let Comms Manager choose rate to use
uint8 MM1=1     # If device is a micromodem, use rate 1
uint8 MM4=4     # If device is a micromodem, use rate 4
uint8 MM5=5     # If device is a micromodem, use rate 5