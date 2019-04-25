cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_sensor )

find_package( catkin REQUIRED COMPONENTS 
    zeabus_library_cpp 
    roscpp
)

catkin_package(
    INCLUDE
    LIBRARIES zeabus_library_cpp roscpp
    DEPENDS Boost
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
