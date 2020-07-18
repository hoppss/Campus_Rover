cmake_minimum_required(VERSION 2.8.3)
project(campusrover_path)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  message_generation
  costmap_2d
  nav_core
  geometry_msgs
  base_local_planner
)


set(CMAKE_CXX_FLAGS "-std=c++11")

add_service_files(FILES
  SavePath.srv
)

generate_messages(DEPENDENCIES std_msgs )

catkin_package(CATKIN_DEPENDS std_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(path_saver src/path_saver.cpp)
target_link_libraries(path_saver ${catkin_LIBRARIES})

add_executable(path_loader src/path_loader.cpp)
target_link_libraries(path_loader ${catkin_LIBRARIES})
