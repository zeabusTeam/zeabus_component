// FILE			: quaternion_filter.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

#include    <iostream>

#include    <zeabus/ros_interfaces/file/base_class.hpp>

#include    <ros/ros.h>

#include    <zeabus/convert/to_string.hpp>

#include    <zeabus/ros_interfaces/convert/geometry_msgs.hpp>

#include    <geometry_msgs/Quaternion.h>

#ifndef _ZEABUS_ROS_INTERFACES_FILE_QUATERNION_FILTER_HPP
#define _ZEABUS_ROS_INTERFACES_FILE_QUATERNION_FILTER_HPP

namespace zeabus
{

namespace ros_interfaces
{

namespace file
{

    class QuaternionFilter : public BaseClass
    {
        public:
            QuaternionFilter( std::string full_path = "" );

            void write_column( std::string input_x = "input_w" , std::string input_y = "input_x"
                    , std::string input_z = "input_y" , std::string input_w = "input_z"
                    , std::string output_x = "output_w" , std::string output_y = "output_x"
                    , std::string output_z = "output_y" , std::string output_w = "output_z");

            void logging( const ros::Time* stamp , const geometry_msgs::Quaternion* input 
                    , const geometry_msgs::Quaternion* output );
            void logging( const ros::Time stamp , const geometry_msgs::Quaternion input
                    , const geometry_msgs::Quaternion output );
    };   

} // namespace file

} // namespace ros_interfaces

} // namespace zeabus

#endif
