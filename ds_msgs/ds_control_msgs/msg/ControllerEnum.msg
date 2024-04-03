#
# Preset allocation modes have their type ID's defined here.
# When building a new custom mode for testing you should
# start at an index of USER_DEFINED + 1 to make sure
# you don't clash with existing allocations.
#
#

uint32 NONE          = 0
uint32 JOYSTICK      = 1
uint32 SURVEY        = 2
uint32 ROV           = 3
uint32 MANUAL        = 4
uint32 USER_DEFINED  = 65535
