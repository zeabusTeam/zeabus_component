// FILE			: trimed_imu.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
//  _LOG_FILTER_    :   This will collect all log of filter
//  _COLLECT_LOG_   :   This will collect all log in this code
//  _LOG_IN_OUT_    :   This will collect all log about pair data of input and output

// README

// REFERENCE

// MACRO SET
//#define _LOG_FILTER_
//#define _LOG_IN_OUT_
//#define _COLLECT_LOG_

// MACRO COMMAND
#ifdef _COLLECT_LOG_
    #define _LOG_FILTER_
    #define _LOG_IN_OUT_
#endif

#include    <iostream>

#include    <zeabus/ros_interfaces/singled_thread.hpp>

#include    <zeabus/client/single_thread/get_sensor_imu.hpp>

#include    <zeabus/service/get_single_data/sensor_imu.hpp>

#include    <geometry_msgs/sensor_imu.h>

#include    <zeabus/escape_code.hpp>

#include    <zeabus/time.hpp>

#ifdef _LOG_FILTER_
#include    <zeabus/ros_interfaces/file/vector3_filter.hpp>
#endif

#ifdef _LOG_IN_OUT_
#include    <zeabus/ros_interfaces/file/quaternion_filter.hpp>
#endif

// Part of algoritm
#include    <zeabus/filter/trimed_mean.hpp>

int main( int argv , char** argc )
{

} // function main
