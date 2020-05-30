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

add_executable(path_following src/path_following.cpp)
target_link_libraries(path_following ${catkin_LIBRARIES})


