cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(FAST_GLOBAL_REGISTRATION)
set (CMAKE_CXX_STANDARD 11)
include_directories(${CMAKE_SOURCE_DIR}/inc)

find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(PCL 1.8 COMPONENTS common io features kdtree REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

set(SOURCES src/main.cpp src/fgr_options.cpp src/fast_global_registration.cpp src/CPUTimer.cpp)
set(PCL_LIBS ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_SEARCH_LIBRARIES} ${PCL_KDTREE_LIBRARIES})
set(LIBS ${PCL_LIBS} ${Boost_PROGRAM_OPTIONS_LIBRARY})

add_executable(FastGlobalRegistration ${SOURCES})
target_link_libraries(FastGlobalRegistration ${LIBS})