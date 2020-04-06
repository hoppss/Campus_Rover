cmake_minimum_required(VERSION 2.8.3)
project(autolabor_demo)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  autolabor_sensors
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  gps_common
  autolabor_msgs
  ackermann_msgs
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pev_demo
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(demo_gps src/demo_gps.cpp)
target_link_libraries(demo_gps
  ${catkin_LIBRARIES}
)

add_executable(rviz_goal_gps test/rviz_goal_gps.cpp)
target_link_libraries(rviz_goal_gps
  ${catkin_LIBRARIES}
)
