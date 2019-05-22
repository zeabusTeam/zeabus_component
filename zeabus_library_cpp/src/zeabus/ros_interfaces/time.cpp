// FILE			: time.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 22 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

#include    <zeabus/ros_interfaces/time.hpp>

namespace zeabus
{

namespace ros_interfaces
{

    std::string string()
    {
        ros::Time time = ros::Time::now();
        return zeabus::convert::to_string( time.sec ) + "."
                + zeabus::convert::to_string( time.nsec );
    }

} // namespace ros_interfaces

} // namespace zeabus
