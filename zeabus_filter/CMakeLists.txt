cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_filter )

set( CMAKE_CXX_FLAGS "-Wall -std=c++14 -pthread" )

find_package( catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    tf
)

find_package( zeabus_library_cpp REQUIRED )
find_package( zeabus_utility REQUIRED )

catkin_package(
    CATKIN_DEPENDS roscpp tf
)

#message("Zeabus utility include dir is ${zeabus_utility_INCLUDE_DIRS}")

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_cpp_INCLUDE_DIRS}
    ${zeabus_utility_INCLUDE_DIRS}
)

add_executable( pressure_filter src/trimed_pressure.cpp )
target_link_libraries( pressure_filter
    ${zeabus_library_cpp_LIBRARIES} 
    ${catkin_LIBRARIES}
)

add_executable( imu_filter src/trimed_imu.cpp )
target_link_libraries( imu_filter
    ${zeabus_library_cpp_LIBRARIES} 
    ${catkin_LIBRARIES}
)

add_executable( dvl_filter src/trimed_dvl.cpp )
target_link_libraries( dvl_filter
    ${zeabus_library_cpp_LIBRARIES} 
    ${catkin_LIBRARIES}
)
