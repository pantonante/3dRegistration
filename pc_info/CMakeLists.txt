cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pc_info)

find_package(PCL 1.8 REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pc_info main.cpp)
target_link_libraries (pc_info ${PCL_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY})