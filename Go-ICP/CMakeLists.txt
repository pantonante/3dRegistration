############################################
# Global ICP (GO-ICP)
############################################

cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project( goicp VERSION 0.1.0 LANGUAGES CXX )
set(CMAKE_CXX_STANDARD 14)

if(DEFINED ENV{A17_ROOT})
  # If goicp is not build from the top level, this is needed to find certain
  # things (e.g. Catch)
  include("$ENV{A17_ROOT}/cmake/A17DefaultPaths.cmake")
endif()

#--------------------------------------------------------------------------------
# Definitions
#add_definitions(-Wall)

# ------------------------------------------------------------------------------
# Dependencies

if(NOT TARGET pcl)
  find_package(PCL 1.8.0 REQUIRED QUIET)

  # Bug-fix
  # If compiling you have the following error:
  # c++: error:  -march=native -msse4.2 -mfpmath=sse : No such file or directory
  # it is caused by a bug in the PCL_DEFINITION variable.
  # PCL_DEFINITION should be a list of words separated by semicolons , i.e.
  # "-march=native;-msse4.2;-mfpmath=sse", while it comes separated by spaces.
  string(STRIP "${PCL_DEFINITIONS}" FIX_PCL_DEFINITIONS) # Removes leading and trailing spaces
  string(REPLACE " " ";" FIX_PCL_DEFINITIONS "${FIX_PCL_DEFINITIONS}")
  
  set( PCL_ROOT $ENV{PCL_ROOT} )
  add_library(pcl INTERFACE IMPORTED)
  # PCL sets some naughty values in their libraries list. Remove them.
  list(REMOVE_ITEM PCL_LIBRARIES "optimized" "debug")
  set_target_properties(pcl PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${PCL_LIBRARIES}"
    INTERFACE_COMPILE_OPTIONS "${FIX_PCL_DEFINITIONS}")
endif()

if(NOT TARGET eigen3)
  find_package(Eigen3 REQUIRED)
  add_library(eigen3 INTERFACE IMPORTED)
  set_target_properties(eigen3 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()

if(NOT TARGET boost::program_options)
  find_package(Boost REQUIRED COMPONENTS program_options)
  add_library(boost::program_options INTERFACE IMPORTED)
  set_target_properties(boost::program_options PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${Boost_PROGRAM_OPTIONS_LIBRARIES}")
endif()

find_package(spdlog REQUIRED CONFIG)

# if(NOT TARGET catch)
#   find_package(Catch REQUIRED)
#   add_library(catch INTERFACE IMPORTED)
#   set_target_properties(catch PROPERTIES
#     INTERFACE_INCLUDE_DIRECTORIES "${CATCH_INCLUDE_DIRS}")
# endif()

#--------------------------------------------------------------------------------
# Build goicp
add_executable( goicp 
  distance_transform.cpp
  goicp_options.cpp
  goicp.cpp
  main.cpp
)

# target_include_directories( goicp  PUBLIC
#   ${CMAKE_CURRENT_BINARY_DIR}
# )

target_link_libraries( goicp 
  boost::program_options
  eigen3
  pcl
  spdlog::spdlog
)

install(TARGETS goicp DESTINATION "bin")

# # ------------------------------------------------------------------------------
# # unittests executable
# set(TEST_NAME unittests_${PROJECT_NAME})
# add_executable(${TEST_NAME}
#   "data_parsing_test.cpp"
#   "data_parsing.cpp"
#   "unittests_main.cpp")
# target_include_directories(${TEST_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
#   $<BUILD_INTERFACE:${workspace_root}>)
# target_link_libraries(${TEST_NAME}
#   catch
#   eigen3)

# file(COPY test_resources DESTINATION .)

# # ------------------------------------------------------------------------------
# # Unit tests
# enable_testing()
# add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})