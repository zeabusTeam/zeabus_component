cmake_minimum_required(VERSION 2.8.3 )
project( zeabus_manager )

find_package( catkin REQUIRED COMPONENTS
    rospy
    zeabus_utility
    zeabus_library_python
)

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

set( PYTHON_CODE
)

catkin_install_python(
    PROGRAMS ${PYTHON_CODE}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
