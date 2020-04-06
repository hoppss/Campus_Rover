cmake_minimum_required(VERSION 2.8.3)
project(autolabor_routing C CXX)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_CURRENT_BINARY_DIR AND NOT MSVC_IDE)
  message(FATAL_ERROR "In-source builds are not allowed.
Please create a directory and run cmake from there, passing the path to this source directory as the last argument.
This process created the file `CMakeCache.txt' and the directory `CMakeFiles'. Please delete them.")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(bitness 32)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(bitness 64)
  message(STATUS "Building on a 64 bit system")
else()
  message(STATUS "Building on a 32 bit system")
endif()

if(WIN32 AND MSVC_VERSION LESS 1900)
  message(FATAL_ERROR "Building with Microsoft compiler needs Latest Visual Studio 2015 (Community or better)")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  # pev_odom
  gps_common
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pev_osrm
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${LibOSRM_LIBRARY_DIRS})

# for RVIZ interactive_markers
add_executable(osrm_routing src/osrm_routing.cpp)

find_package(LibOSRM REQUIRED)
target_link_libraries(osrm_routing ${catkin_LIBRARIES} ${LibOSRM_LIBRARIES} ${LibOSRM_DEPENDENT_LIBRARIES})

include_directories(SYSTEM ${LibOSRM_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LibOSRM_CXXFLAGS}")