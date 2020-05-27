cmake_minimum_required(VERSION 2.8.3)
project(campusrover_arm_move)

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
  campusrover_msgs
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  apriltag_ros
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES campusrover_arm_move
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

add_executable(apriltag_to_press src/apriltag_to_press.cpp)
target_link_libraries(apriltag_to_press  ${catkin_LIBRARIES})
