cmake_minimum_required(VERSION 2.8.3)
project(campusrover_elevator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf2_ros
  navfn
  sensor_msgs
  nav_core
  control_toolbox
  message_generation
  tf2_geometry_msgs
  laser_geometry
  campusrover_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES campusrover_elevator
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

#
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(elevator_position_finder src/elevator_position_finder.cpp)
target_link_libraries(elevator_position_finder ${catkin_LIBRARIES})

add_executable(elevator_pose_tf src/elevator_pose_tf.cpp)
target_link_libraries(elevator_pose_tf ${catkin_LIBRARIES})

add_executable(elevator_door_status src/elevator_door_status.cpp)
target_link_libraries(elevator_door_status ${catkin_LIBRARIES})

add_executable(elevator_floor_status src/elevator_floor_status.cpp)
target_link_libraries(elevator_floor_status ${catkin_LIBRARIES})

add_executable(elevator_path_generater src/elevator_path_generater.cpp)
target_link_libraries(elevator_path_generater ${catkin_LIBRARIES})

add_executable(elevator_status_controller src/elevator_status_controller.cpp)
target_link_libraries(elevator_status_controller ${catkin_LIBRARIES})
