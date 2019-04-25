cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_sensor )

find_package( catkin REQUIRED COMPONENTS 
    zeabus_library_cpp 
    roscpp
    geometry_msgs
)

catkin_package(
    LIBRARIES zeabus_library_cpp roscpp geometry_msgs
    DEPENDS Boost
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable( imu_node src/imu_node.cpp)
