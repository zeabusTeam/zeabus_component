// FILE			: vector3_filter.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 18 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

#include    <zeabus/ros_interfaces/file/base_class.hpp>

#include    <zeabus/convert/to_string.hpp>

#include    <zeabus/ros_interfaces/convert/geometry_msgs.hpp>

#include    <geometry_msgs/Vector3.h>

#ifndef _ZEABUS_ROS_INTERFACES_FILE_VECTOR3_FILTER_HPP__
#define _ZEABUS_ROS_INTERFACES_FILE_VECTOR3_FILTER_HPP__

namespace zeabus
{

namespace ros_interfaces
{

namespace file
{

    class Vector3Filter : public  BaseClass
    {
        public:
            Vector3Filter( std::string full_path = "" );

            void write_column( std::string input_x = "input_x" , std::string input_y = "input_y"
                    , std::string input_z = "input_z" , std::string output_x = "output_x" 
                    , std::string output_y = "output_y" , std::string output_z = "output_z" );

            void logging( const ros::Time* stamp , const geometry_msgs::Vector3* input 
                    , const geometry_msgs::Vector3* output );
            void logging( const ros::Time stamp , const geometry_msgs::Vector3 input
                    , const geometry_msgs::Vector3 output );
    };

} // namespace file

} // namespace ros_interface

} // namespace zeabus

#endif // _ZEABUS_ROS_INTERFACES_FILE_VECTOR3_FILTER_HPP__
