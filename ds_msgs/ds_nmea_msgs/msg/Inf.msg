# $PHINF,hhhhhhhh*hh
#Bit 0 Heading not valid
#Bit 1 Roll not valid
#Bit 2 Pitch not valid
#Bit 3 Position (Heave, surge and sway) not valid
#Bit 4 Position calculation starting
#Bit 5 Initialization
#Bit 6 Reserved
#Bit 7 Reserved
#Bit 8 FOG X1 Error
#Bit 9 FOG X2 Error
#Bit 10 FOG X3 Error
#Bit 11 Optical source error
#Bit 12 Accelerometer X1 error
#Bit 13 Accelerometer X2 error
#Bit 14 Accelerometer X3 error
#Bit 15 Analog input A or B error
#Bit 16 Serial input A error
#Bit 17 Serial input B error
#Bit 18 Serial input C error
#Bit 19 FIFO Full
#Bit 20 Serial output A full
#Bit 21 Serial output B full
#Bit 22 Serial output C full
#Bit 23 Reserved
#Bit 24 Manual log used
#Bit 25 Manual latitude used

bool heading_invalid
bool pitch_invalid
bool roll_invalid
uint8 checksum