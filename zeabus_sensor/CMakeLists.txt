cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_sensor )

find_package( catkin REQUIRED COMPONENTS 
    roscpp
    geometry_msgs
    sensor_msgs
)

find_package( zeabus_library_cpp REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp
)

message("zeabus_library_cpp_INCLUDE_DIRS is ${zeabus_library_cpp_INCLUDE_DIRS}")
message("catkin_INCLUDE_DIRS is ${catkin_INCLUDE_DIRS}")

message("zeabus_library_cpp_LIBRARIES is ${zeabus_library_cpp_LIBRARIES}")
message("catkin_LIBRARIES is ${catkin_LIBRARIES}")

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_cpp_INCLUDE_DIRS}
)

add_executable( imu_node src/imu_node.cpp)
add_dependencies( imu_node ${zeabus_library_cpp_LIBRARIES} ${catkin_LIBRARIES} )
target_link_libraries( imu_node 
    ${zeabus_library_cpp_LIBRARIES} 
    ${catkin_LIBRARIES}
)
