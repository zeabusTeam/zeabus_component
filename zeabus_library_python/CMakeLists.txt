cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_library_python )

## link reference about CMake to make file python
#   ref1    : http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile?fbclid=IwAR2A9UVzczVQoIe31XKCcw6TY180kRGY0WTpahvKoLr72S99Lrg5FKCUa9A

find_package( catkin REQUIRED COMPONENTS rospy )

catkin_package()

catkin_python_setup()

catkin_install_python( PROGRAMS 
    bin/zeabus_lib_python.py
    bin/zeabus_test_command.py
    bin/zeabus_analysis_vision.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
