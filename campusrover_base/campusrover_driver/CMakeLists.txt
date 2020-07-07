cmake_minimum_required(VERSION 2.8.3)
project(campusrover_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  dynamic_reconfigure
)



################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Encode.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages()
generate_dynamic_reconfigure_options(cfg/SimCampusroverDriver.cfg)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES campusrover_driver
CATKIN_DEPENDS roscpp tf message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(campusrover_driver src/campusrover_driver.cpp)
target_link_libraries(campusrover_driver ${catkin_LIBRARIES})

add_executable(joy_to_twist src/joy_to_twist.cpp)
target_link_libraries(joy_to_twist ${catkin_LIBRARIES}) 

add_executable(luke_cmd_vel_mux src/luke_cmd_vel_mux.cpp)
target_link_libraries(luke_cmd_vel_mux ${catkin_LIBRARIES})



#############
## Install ##
#############



#############
## Testing ##
#############


