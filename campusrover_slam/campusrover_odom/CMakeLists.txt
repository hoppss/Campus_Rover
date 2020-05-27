cmake_minimum_required(VERSION 2.8.3)
project(campusrover_odom)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_filters
  nav_msgs
  sensor_msgs
  tf
  geometry_msgs
  gps_common
  campusrover_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(campusrover_odom src/campusrover_odom.cpp)
target_link_libraries( campusrover_odom ${catkin_LIBRARIES} )