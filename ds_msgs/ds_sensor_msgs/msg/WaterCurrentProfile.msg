#Message format for Water Current Velocity Information at each depth bin


#if set to beam: the size of this velocity_bin_beam array is 4 (x,y,z velocity in each bin relative to each individual beam)
#if set to instrument,ship, earth: the size of this velocity_bin_beam array is 1 (x,y,z velocity in each bin relative to instrument frame)

geometry_msgs/Vector3[] velocity_bin_beam




#Intensity at each bin IN [dB]
#uint32[] bin_intensity

#Correlation Factor --> [0,255]
#uint16[] bin_correlation