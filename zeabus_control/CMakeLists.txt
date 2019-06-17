cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_control )

set( CMAKE_CXX_FLAGS "-Wall -std=c++14 -pthread")

find_package( catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    tf
)

find_package( zeabus_library_cpp REQUIRED )
find_package( zeabus_utility REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp tf
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_cpp_INCLUDE_DIRS}
    "${PROJECT_SOURCE_DIR}/include"
)

add_executable( interfaces
    src/interfaces.cpp
)
add_dependencies( interfaces ${catkin_EXPORTED_TARGETS} )
target_link_libraries( interfaces 
    ${zeabus_library_cpp_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable( control_fuzzy
    src/control_fuzzy.cpp
)
add_dependencies( control_fuzzy ${catkin_EXPORTED_TARGETS} )
target_link_libraries( control_fuzzy 
    ${zeabus_library_cpp_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable( fuzzy
    src/fuzzy.cpp
)
add_dependencies( fuzzy ${catkin_EXPORTED_TARGETS} )
target_link_libraries( fuzzy 
    ${zeabus_library_cpp_LIBRARIES}
    ${catkin_LIBRARIES}
)