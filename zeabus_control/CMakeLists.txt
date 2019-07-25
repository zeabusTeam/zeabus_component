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

add_executable( main_control
    src/main_control.cpp
)
add_dependencies( main_control ${catkin_EXPORTED_TARGETS} )
target_link_libraries( main_control 
    ${zeabus_library_cpp_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable( velocity_interfaces
    src/velocity_interfaces.cpp
)
add_dependencies( velocity_interfaces ${catkin_EXPORTED_TARGETS} )
target_link_libraries( velocity_interfaces 
    ${zeabus_library_cpp_LIBRARIES}
    ${catkin_LIBRARIES}
)

<<<<<<< HEAD
=======
<<<<<<< HEAD
add_executable( control_by_force
    src/control_by_force.cpp
)
add_dependencies( control_by_force ${catkin_EXPORTED_TARGETS} )
target_link_libraries( control_by_force
=======
>>>>>>> e7f55e878a501a34601672a99b014ec52c07c371
#add_executable( control_velocity
#    src/control_velocity.cpp
#)
#add_dependencies( control_velocity ${catkin_EXPORTED_TARGETS} )
#target_link_libraries( control_velocity
#    ${zeabus_library_cpp_LIBRARIES}
#    ${catkin_LIBRARIES}
#)

add_executable( control_velocity_output
    src/control_velocity_output.cpp
)
add_dependencies( control_velocity_output ${catkin_EXPORTED_TARGETS} )
target_link_libraries( control_velocity_output
    ${zeabus_library_cpp_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable( test_lib_fuzzy test/fuzzy_3_dimension.cpp)
add_dependencies( test_lib_fuzzy ${catkin_EXPORTED_TARGETS})
target_link_libraries( test_lib_fuzzy
<<<<<<< HEAD
=======
>>>>>>> 6ff860c33f1f9a1f6c049c896f2220b814c6b50b
>>>>>>> e7f55e878a501a34601672a99b014ec52c07c371
    ${zeabus_library_cpp_LIBRARIES}
    ${catkin_LIBRARIES}
)
