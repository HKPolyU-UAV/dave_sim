# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

string ip_addr
string netmask
string brdcst
string dfltgw
string dfltip
string dflmac
float64 dflprt
string laripa
string larmac
float64 larprt
string lfpipa
string lfpmac
float64 lfpprt 
string remipa
string raripa
string rarmac
string rsubnt
uint16 brdgen
string brdgip
string guiipa 
string fpg3vr
string fpg6vr
string softvr
uint16 reboot
uint16 dfmode
uint16 master_slaven
uint16 logena
uint16 acsten
uint16 acstmast_slaven
uint16 acstmfg
uint16 lamdid
uint16 ramdid
uint16 sensitivity_ctrl
uint16 lrhaln
uint16 status1hz_enable