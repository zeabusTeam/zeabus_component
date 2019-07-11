cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_sensor )

set( CMAKE_CXX_FLAGS "-Wall -std=c++14 -pthread" )

find_package( catkin REQUIRED COMPONENTS 
    roscpp
    geometry_msgs
    sensor_msgs
)

find_package( zeabus_library_cpp REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp
)

#message("zeabus_library_cpp_INCLUDE_DIRS is ${zeabus_library_cpp_INCLUDE_DIRS}")
#message("catkin_INCLUDE_DIRS is ${catkin_INCLUDE_DIRS}")

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_cpp_INCLUDE_DIRS}
)

add_executable( imu_node src/imu_node.cpp)
target_link_libraries( imu_node 
    ${zeabus_library_cpp_LIBRARIES} 
    ${catkin_LIBRARIES}
)

add_executable( capture_gyro_bias src/imu_capture_gyro.cpp)
target_link_libraries( capture_gyro_bias 
    ${zeabus_library_cpp_LIBRARIES} 
    ${catkin_LIBRARIES}
)
add_executable( dvl_node src/dvl_node.cpp)
target_link_libraries( dvl_node 
    ${zeabus_library_cpp_LIBRARIES} 
    ${catkin_LIBRARIES}
)
