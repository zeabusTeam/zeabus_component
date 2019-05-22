cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_library_cpp )

## link reference about CMake 
#   ref1    : https://cmake.org/cmake/help/v2.8.3/cmake.html
#   ref2    : https://ecrafter.wordpress.com/2012/04/24/cmake-tutorial-part-2/

## linke reference about what is catkin_make
#   ref     : http://wiki.ros.org/catkin/commands/catkin_make


## find_package this importance
#   That make us to ensure which package has been make or build before us

find_package( catkin REQUIRED geometry_msgs roscpp sensor_msgs zeabus_utility roslib tf)
find_package( Boost REQUIRED COMPONENTS )
find_package( zeabus_utility REQUIRED )

set( SHARED_LIBS 
    Sort
    Array
    Convert
    General
    IMUConnector
    DVLConnector
    ClientFusion
    GeneralService
    SynchronousPort
    MAESTROConnector
    ClientSingleThread
    ServiceGetData
)

set( ROS_LIBS
    ROSConvert
    ROSInterfaces
    ROSSingleFilter
    ROSVector3Filter
    ROSQuaternionFilter
)

set( zeabus_library_include_path 
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/dynamic_src"
)

catkin_package(
    # INCLUDE_DIRS will help you to export header
    INCLUDE_DIRS ${zeabus_library_include_path} 
    # LIBRARIES will help you to export library
    LIBRARIES ${SHARED_LIBS} ${ROS_LIBS}
    # We help you link on syntex of catkin_EXPORTE_TARGEETS
    CATKIN_DEPENDS zeabus_utility roscpp tf
    DEPENDS Boost
)

set( CMAKE_CXX_FLAGS "-Wall -std=c++14")

include_directories(
    "${zeabus_library_include_path}"
    "${zeabus_utility_INCLUDE_DIRS}"
    ${catkin_INCLUDE_DIRS} # this line will link to catkin_header
    ${Boost_INCLUDE_DIR}
)

#message("CMAKE_CXX_FLAGS is ${CMAKE_CXX_FLAGS}")
#message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
#message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
#message("CMAKE_CXX_STANDARD is ${CMAKE_CXX_STANDARD}")

#message("zeabus_utility_INCLUDE_DIRS is ${zeabus_utility_INCLUDE_DIRS}")
#message("zeabus_LIBRARYS is ${zeabus_LIBRARYS}  ")

add_subdirectory( src/zeabus )
