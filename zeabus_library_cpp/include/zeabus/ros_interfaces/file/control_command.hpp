// FILE			: control_command.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 25 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  Please red ref1 to know type of array of msg in c++

// REFERENCE
//  ref1    : http://wiki.ros.org/msg

// MACRO SET

// MARCO CONDITION

#include    <iostream>

#include    <vector>

#include    <ros/ros.h>

#include    <zeabus/convert/to_string.hpp>

#include    <zeabus/ros_interfaces/file/base_class.hpp>

#ifndef _ZEABUS_ROS_INTERFACES_FILE_CONTROL_COMMAND_HPP__
#define _ZEABUS_ROS_INTERFACES_FILE_CONTROL_COMMAND_HPP__

namespace zeabus
{

namespace ros_interfaces
{

namespace file
{

    class ControlCommand : public BaseClass
    {
        public:
            ControlCommand( std::string full_path = "" );

            void write_column( std::string* num_part , std::string* bool_part );

            void logging( const ros::Time* stamp , const std::vector< double >* num_part 
                    , const std::vector<uint8_t>* bool_part );
            void logging( const ros::Time stamp , const std::vector< double > num_part
                    , const std::vector<uint8_t> bool_part );
    }; // object ControlCommand

} // namespace file

} // namespace ros_interfaces

} // namespace zeabus

#endif
