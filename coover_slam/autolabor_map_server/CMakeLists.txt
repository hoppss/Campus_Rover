cmake_minimum_required(VERSION 2.8.3)
project(autolabor_map_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  genmsg
  # map_server
  laser_geometry
  pcl_conversions
  pcl_ros
  # for RVIZ interactive_markers
  interactive_markers
  visualization_msgs
  gps_common
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()

# using for build panel
## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
# if(rviz_QT_VERSION VERSION_LESS "5")
#   message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#   find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
#   ## pull in all required include dirs, define QT_LIBRARIES, etc.
#   include(${QT_USE_FILE})
# else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
# endif()


# For using Boost
FIND_PACKAGE(Boost)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

set(CMAKE_CXX_FLAGS "-std=c++11")
include_directories(/usr/include/cppconn)
link_directories(/usr/lib)

add_service_files(FILES
  DatabaseWrite.srv
  MapUpdate.srv
  SavePcd.srv
)

generate_messages(DEPENDENCIES)

catkin_package()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIR}
  ${SDL_IMAGE_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
)

add_definitions(-DQT_NO_KEYWORDS)

set(MAP_TOOL_SRC
  src/map_tool_panel.cpp
)
add_library(autolabor_rviz_panels ${MAP_TOOL_SRC})
target_link_libraries(autolabor_rviz_panels ${QT_LIBRARIES} ${catkin_LIBRARIES})

# add_executable(map_tf_finder src/map_tf_finder.cpp)
# target_link_libraries(map_tf_finder ${catkin_LIBRARIES} mysqlcppconn)
# add_dependencies(map_tf_finder autolabor_map_server_gencpp)
#
# add_executable(autolabor_map_server src/autolabor_map_server.cpp)
# target_link_libraries(autolabor_map_server
#                       ${catkin_LIBRARIES}
#                       mysqlcppconn
#                       ${YAMLCPP_LIBRARIES}
#                       ${catkin_LIBRARIES})
# add_dependencies(autolabor_map_server autolabor_map_server_gencpp)
#
# add_executable(scan_to_pointcloud2 src/scan_to_pointcloud2.cpp)
# target_link_libraries(scan_to_pointcloud2 ${catkin_LIBRARIES})

add_executable(autolabor_3d_map src/autolabor_3d_map.cpp)
target_link_libraries(autolabor_3d_map ${catkin_LIBRARIES})

add_executable(pcd_publish src/pcd_publish.cpp)
target_link_libraries(pcd_publish ${catkin_LIBRARIES})

add_executable(pcd_to_pgm_map src/pcd_to_pgm_map.cpp)
target_link_libraries(pcd_to_pgm_map ${catkin_LIBRARIES})

# for RVIZ interactive_markers
add_executable(pcd_tf_finder src/pcd_tf_finder.cpp)
target_link_libraries(pcd_tf_finder ${catkin_LIBRARIES} yaml-cpp)

# for RVIZ interactive_markers
add_executable(autolabor_pcd_tf_finder src/autolabor_pcd_tf_finder.cpp)
target_link_libraries(autolabor_pcd_tf_finder ${catkin_LIBRARIES})

# PCD map loader
add_executable(pcd_map_loader src/pcd_map_loader.cpp)
target_link_libraries(pcd_map_loader ${catkin_LIBRARIES} yaml-cpp)

add_executable(gpx_export src/gpx_export.cpp)
target_link_libraries(gpx_export ${catkin_LIBRARIES})