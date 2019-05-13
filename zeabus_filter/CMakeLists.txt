cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_filter )

set( CMAKE_CXX_FLAGS "-Wall -std=c++14 -pthread" )

find_package( catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
)

find_package( zeabus_library_cpp REQUIRED )

catkin_package(
    CATKIN_DEPENDS roscpp
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_cpp_INCLUDE_DIRS}
)
