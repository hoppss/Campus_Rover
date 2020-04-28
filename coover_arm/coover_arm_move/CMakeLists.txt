cmake_minimum_required(VERSION 2.8.3)
project(coover_arm_move)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  roscpp
  rospy
  autolabor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coover_arm_move
#  CATKIN_DEPENDS moveit_core roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(position_moveit src/position_moveit.cpp)
target_link_libraries(position_moveit  ${catkin_LIBRARIES})

add_executable(position_moveit3 src/position_moveit3.cpp)
target_link_libraries(position_moveit3  ${catkin_LIBRARIES})
