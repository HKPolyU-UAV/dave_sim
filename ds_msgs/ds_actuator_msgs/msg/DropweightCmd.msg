time stamp

int32 IDX_DESCENT=0
int32 IDX_ASCENT=1
int32 IDX_ANCHOR=2
int32 IDX_MAX=8

# This is an array of dropweight booleans.  If the bool at a particular index
# is true, the weight should be dropped.
bool[8] drop

# burn is an array of burnwire booleans, one for each weight.  This message assumes
# each vehicle supports both a primary dropweight actuator (dcam, etc) and a backup
# burnwire.
bool[8] burn
