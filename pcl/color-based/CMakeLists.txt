cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(region_growing)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pvso region_growing.cpp)
target_link_libraries (pvso ${PCL_LIBRARIES})