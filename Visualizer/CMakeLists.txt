cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(VIZ)
#include_directories(${CMAKE_SOURCE_DIR}/inc)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

set(SOURCES src/viz.cpp)
set(LIBS ${PCL_LIBRARIES} )
add_executable(viz ${SOURCES})
target_link_libraries(viz ${LIBS})