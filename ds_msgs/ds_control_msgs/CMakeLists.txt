cmake_minimum_required(VERSION 2.8.3)
project(ds_control_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  control_msgs
  ds_core_msgs
  ds_nav_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  BottomFollow1D.msg
  GoalLegState.msg
  GoalLegLatLon.msg
  ExternalBottomFollowAlarm.msg
  ExternalBottomFollowTimedOverride.msg
  RovAutoState.msg
  RovControllerState.msg
  RovControlGoal.msg
  ActuatorInputs.msg
  ControllerEnum.msg
  JoystickEnum.msg
  PidSettings.msg
  RovAllocationEnum.msg
  RovSwitchState.msg
)

# Generate services
add_service_files(
  DIRECTORY srv
  FILES
  RovAutoCommand.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 ds_core_msgs
 geometry_msgs
 control_msgs
 ds_nav_msgs
)

catkin_package(
   CATKIN_DEPENDS message_generation std_msgs ds_core_msgs geometry_msgs control_msgs
 ds_nav_msgs
 geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

