# This message is output by processes considered critical,
# To be used by abort decision logic. If the abort
# supervisor stops hearing from a critical process,
# then it should abort
std_msgs/Header header

string nodename
int32 ttl
