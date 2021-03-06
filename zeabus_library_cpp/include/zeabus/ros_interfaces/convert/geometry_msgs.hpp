// FILE			: geometry_msgs.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 18 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This header will manage about conversation of geometry_msgs in ros system
//  Design of function name to have pattern is
//          ${source}_${target}

// REFERENCE
//  ref1    : https://www.geeksforgeeks.org/const-member-functions-c/
//  ref2    : http://docs.ros.org/melodic/api/tf/html/c++/classtf_1_1Quaternion.html

// MACRO SET

#include    <zeabus/convert/to_string.hpp>

#include    <tf/LinearMath/Quaternion.h>

#include    <geometry_msgs/Vector3.h>

#include    <geometry_msgs/Quaternion.h>

#ifndef _ZEABUS_ROS_INTERFACES_CONVERT_GEOMETRY_MSGS_HPP
#define _ZEABUS_ROS_INTERFACES_CONVERT_GEOMETRY_MSGS_HPP

namespace zeabus
{

namespace ros_interfaces
{

namespace convert
{

    std::string vector3_string( const geometry_msgs::Vector3* data , const char delim );
    std::string vector3_string( const geometry_msgs::Vector3 data , const char delim ); 

    std::string quaternion_string( const geometry_msgs::Quaternion* data , const char delim );
    std::string quaternion_string( const geometry_msgs::Quaternion data , const char delim );

    void tf_quaternion( const tf::Quaternion* source , geometry_msgs::Quaternion* target );
    void quaternion_tf( const geometry_msgs::Quaternion* source , tf::Quaternion* target );

    void vector3_quaternion( const geometry_msgs::Vector3* source , tf::Quaternion* target );
    void quaternion_vector3( const tf::Quaternion* source , geometry_msgs::Vector3* target );

} // namespace convert

} // namespace ros_interfaces

} // namespace zeabus

#endif
