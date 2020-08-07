cmake_minimum_required(VERSION 2.8.3)
project(campusrover_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf2_ros
  nav_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  DoorStatus.msg
  ElevatorControlStatus.msg
  FloorStatus.msg
  Encode.msg
  ButtonCommand.msg
  PoseWithId.msg
  InDemoStatus.msg
  Room.msg
  RoomConnected.msg
  State.msg
  NodeInfo.msg
  WorkingFloor.msg
  Trip.msg
  LocationRoom.msg
  HmiStatus.msg
)
##
## Generate service in the 'srv' folder###
add_service_files(
  FILES
  DriveStatus.srv
  ArmAction.srv
  ArmStandby.srv
  PlannerFunction.srv
  InitFloor.srv
  PressButton.srv
  ElevatorControlInterface.srv
  ElevatorStatusChecker.srv
  ButtonStatus.srv
  PlannerFunction.srv
  MapInfo.srv
  StatePlan.srv
  ModuleInfo.srv
  RoutingPath.srv
  MapLoadInfo.srv
  MapInitPose.srv
  LocalizerSwitch.srv
  ArmTablePosition.srv
  ArmTableHomeReturn.srv
)
#
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pev_msgs
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs message_runtime
#  DEPENDS system_lib
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
