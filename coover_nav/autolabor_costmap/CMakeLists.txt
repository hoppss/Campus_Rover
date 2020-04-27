cmake_minimum_required(VERSION 2.8.3)
project(autolabor_costmap)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  nav_msgs
  laser_geometry
)

find_package (Eigen3 REQUIRED NO_MODULE)
find_package (OpenCV REQUIRED)
find_package (PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autolabor_costmap
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(autolabor_costmap_lib src/autolabor_costmap.cpp)
target_link_libraries( 
  autolabor_costmap_lib
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES})

add_executable(autolabor_costmap src/autolabor_costmap_node.cpp)
target_link_libraries( 
  autolabor_costmap
  autolabor_costmap_lib                 
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES} )

add_executable(laser_to_pointcloud2 src/laser_to_pointcloud2.cpp)
target_link_libraries(laser_to_pointcloud2 ${catkin_LIBRARIES} )
