// FILE			: single_filter.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This log type single input and single output 

// REFERENCE
//  ref01   : http://wiki.ros.org/msg

#include    <iostream>

#include    <zeabus/ros_interfaces/file/base_class.hpp>

#include    <ros/ros.h>

#include    <zeabus/convert/to_string.hpp>

#ifndef _ZEABUS_ROS_INTERFACES_FILE_SINGLE_FILTER_HPP__
#define _ZEABUS_ROS_INTERFACES_FILE_SINGLE_FILTER_HPP__

namespace zeabus
{

namespace ros_interfaces
{

namespace file
{

    class SingleFilter : public BaseClass
    {
        public:
            SingleFilter( std::string full_path = "" );

            // We automatic write time for first paragraph
            void write_column( std::string input_name = "input"
                    , std::string output_name = "output");

            // We don't check you already open file or not
            // Beacause we create interface for manage your input only
            void logging( ros::Time* stamp , double* input , double* output );
            void logging( ros::Time stamp , double input , double output );
    };

} // namespace file

} // namespace ros_interfaces

} // namespace zeabus

#endif // _ZEABUS_ROS_INTERFACES_FILE_SINGLE_FILTER_HPP__
