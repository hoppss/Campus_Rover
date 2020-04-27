cmake_minimum_required(VERSION 2.8.3)
project(ui_tool)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rviz
  geometry_msgs
  visualization_msgs
  gps_common
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system-lib
)

qt5_wrap_cpp(QT_MOC_HPP include/map_tool.hpp
                        include/map_tool_ui.hpp
                        include/gpx_table.hpp )

qt5_wrap_cpp(QT_MOC_HPP_RVIZ include/click_point_rviz.hpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_executable(autolabor_map_tool src/map_tool.cpp
                            src/map_tool_ui.cpp
                            src/map_tool_main.cpp
                            src/gpx_table.cpp
                            ${QT_MOC_HPP}
                            )

target_link_libraries(autolabor_map_tool  ${QT_LIBRARIES}
                                    ${catkin_LIBRARIES}
                                    Qt5::Widgets)

add_library(autolabor_rviz_tools  src/click_point_rviz.cpp
                            ${QT_MOC_HPP_RVIZ})

# target_link_libraries(autolabor_rviz_tools ${QT_LIBRARIES} ${OGRE_LIBRARIES})

target_link_libraries(autolabor_rviz_tools ${QT_LIBRARIES} )

install(TARGETS
  autolabor_rviz_tools
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)