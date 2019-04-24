cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_library_cpp )

## link reference about CMake 
#   ref1    : https://cmake.org/cmake/help/v2.8.3/cmake.html
#   ref2    : https://ecrafter.wordpress.com/2012/04/24/cmake-tutorial-part-2/

## linke reference about what is catkin_make
#   ref     : http://wiki.ros.org/catkin/commands/catkin_make


## find_package this importance
#   That make us to ensure which package has been make or build before us

find_package( catkin REQUIRED )

set( zeabus_library_include_path "${PROJECT_SOURCE_DIR}/include")

include_directories(
    "${zeabus_library_include_path}"
)

message("CMAKE_CXX_FLAGS is ${CMAKE_CXX_FLAGS}")
message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")

add_subdirectory( src/zeabus )
