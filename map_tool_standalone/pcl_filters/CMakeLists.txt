cmake_minimum_required (VERSION 2.6)

project (pcl_filters)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(filters src/pcl_filters.cpp)
target_link_libraries(filters ${PCL_LIBRARIES})
