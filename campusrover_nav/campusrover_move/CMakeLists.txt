cmake_minimum_required(VERSION 2.8.3)
project(campusrover_move)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

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


# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES campusrover_move
#  CATKIN_DEPENDS roscpp rospy
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

add_executable(dp_planner src/dp_planner.cpp)
target_link_libraries(dp_planner ${catkin_LIBRARIES})

add_executable(adp_planner src/adp_planner.cpp)
target_link_libraries(adp_planner ${catkin_LIBRARIES})

add_executable(lu_simulation src/lu_simulation.cpp)
target_link_libraries(lu_simulation ${catkin_LIBRARIES})

add_executable(elevator_position_finder src/elevator_position_finder.cpp)
target_link_libraries(elevator_position_finder ${catkin_LIBRARIES})

add_executable(elevator_pose_tf src/elevator_pose_tf.cpp)
target_link_libraries(elevator_pose_tf ${catkin_LIBRARIES})

add_executable(elevator_door_status src/elevator_door_status.cpp)
target_link_libraries(elevator_door_status ${catkin_LIBRARIES})

add_executable(elevator_path_generater src/elevator_path_generater.cpp)
target_link_libraries(elevator_path_generater ${catkin_LIBRARIES})
