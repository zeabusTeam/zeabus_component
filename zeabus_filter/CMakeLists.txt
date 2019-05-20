cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_filter )

set( CMAKE_CXX_FLAGS "-Wall -std=c++14 -pthread" )

find_package( catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
)

find_package( zeabus_library_cpp REQUIRED )
find_package( zeabus_utility REQUIRED )

catkin_package(
    CATKIN_DEPENDS roscpp
)

#message("Zeabus utility include dir is ${zeabus_utility_INCLUDE_DIRS}")

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_cpp_INCLUDE_DIRS}
    ${zeabus_utility_INCLUDE_DIRS}
)

add_executable( pressure src/trimed_pressure.cpp )
target_link_libraries( pressure
    ${zeabus_library_cpp_LIBRARIES} 
    ${catkin_LIBRARIES}
)
