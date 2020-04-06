cmake_minimum_required(VERSION 2.8.3)
project(autolabor_pro1_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  message_generation
  tf
)

add_message_files(
  FILES
  Encode.msg
)

generate_messages()


catkin_package(CATKIN_DEPENDS roscpp tf message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(autolabor_pro1_driver src/autolabor_driver.cpp)
target_link_libraries(autolabor_pro1_driver ${catkin_LIBRARIES})


