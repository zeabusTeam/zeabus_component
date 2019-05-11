// FILE			: base_class.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  BaseClass this implement for easy to read or write file but don't help you to manage 
//      outline or pattern to writting or reading please you this to implement in highlevel
//      again
//  When you read about operator << and >> you should read ref if you want to understand
//  How to read or write have seperate type of Function to 2 type Formatted and Unformatted read
//      ref3 and 4 for get different of that

// REFERENCE
//  ref1    : https://en.cppreference.com/w/cpp/header/fstream
//  ref2    : https://en.cppreference.com/w/cpp/string/basic_string
//  ref3    : https://en.cppreference.com/w/cpp/named_req/UnformattedOutputFunction
//  ref4    : https://en.cppreference.com/w/cpp/named_req/FormattedOutputFunction
//  ref5    : https://en.cppreference.com/w/cpp/io/basic_ostream/operator_ltlt
//  ref6    : https://en.cppreference.com/w/cpp/io/basic_istream/operator_gtgt

#include    <iostream> // this manage about std::cin , std::cout

#include    <fstream> // this library about input output file

#include    <zeabus/ros_interfaces/file/path_file.hpp>

#ifndef _ZEABUS_ROS_INTERFACES_FILE_BASE_CLASS_HPP__
#define _ZEABUS_ROS_INTERFACES_FILE_BASE_CLASS_HPP__

namespace zeabus
{

namespace ros_interfaces
{

namespace file
{

    class BaseClass : public zeabus::ros_interfaces::file::PathFile
    {
        public:
            BaseClass( std::string full_path = "" );

            // For open if already open file we will auto close previous file for you

            bool open( std::string full_path );
            bool open();

            void close();

            bool is_open();

            // this function will use operator << to write file please read ref 4 & 5
            template< typename type_output >
            void write( type_output* output);

            // this function will use operator >> to read file please read ref 4 & 6
            template< typename type_input >
            void read( type_output* input );

        protected:  
            std::basic_fstrean< char > stream_file;
 
    }; // BaseClass object

} // namespace file

} // namespace ros_interfaces

} // namespace zeabus

#include    <zeabus/ros_interfaces/file/base_class.cpp>

#endif
