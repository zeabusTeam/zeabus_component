cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_fusion )

set( CMAKE_CXX_FLAGS "-Wall -std=c++14 -pthread")

find_package( catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    tf
)

find_package( zeabus_library_cpp REQUIRED )

catkin_package(
    CATKIN_DEPENDS roscpp tf
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_cpp_INCLUDE_DIRS}
)

add_executable( sensor_fusion
    src/raw_fusion.cpp
)
target_link_libraries( sensor_fusion ${catkin_LIBRARIES} ${zeabus_library_cpp_LIBRARIES})
