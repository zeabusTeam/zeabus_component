// FILE			: time.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This will apply ros::Time

// REFERENCE
//  ref1    : https://en.cppreference.com/w/cpp/language/storage_duration 
//  ref2    : https://en.cppreference.com/w/cpp/language/static

// MACRO SET

#include    <ros/time.h>

#include    <zeabus/convert/to_string.hpp>

#ifndef _ZEABUS_ROS_INTERFACES_TIME_HPP__
#define _ZEABUS_ROS_INTERFACES_TIME_HPP__

namespace zeabus
{

namespace ros_interfaces
{

namespace time
{

    // will use ros::Time::now() to convert to string and return result
    std::string time_string();

} // namespace time

} // namespace ros_interfaces

} // namespace zeabus

#endif
