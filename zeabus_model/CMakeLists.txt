cmake_minimum_required( VERSION 2.8.3 )

project( zeabus_model )

find_package( catkin REQUIRED COMPONENTS roscpp tf )

catkin_package(
    CATKIN_DEPENDS roscpp tf
)

include_directories(
    ${catkin_LIBRARIES}
)
