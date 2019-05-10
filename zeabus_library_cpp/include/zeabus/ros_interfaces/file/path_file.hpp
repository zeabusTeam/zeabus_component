// FILE         : path_file.hpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, MAY 10
// MAINTAINER   : K.Supasan

// MACRO DETAIL
//  _PRINT_PROCESS_     : Will print your process to do with this path_file class 

// README
// This code we will use ros for finding path to any package in ros/CMake package.
// We can't use filesystem header of c++ because we have limit compile version 14

// REFERENCE
//  http://docs.ros.org/indigo/api/roslib/html/c++/namespaceros_1_1package.html#ae9470dd201aa4e66abb833e710d812a4
//  https://en.cppreference.com/w/cpp/filesystem/exists

#include    <ros/ros.h>

#include    <iostream>

#include    <ros/package.h> // this header have function to get path to package

#include    <zeabus/escape_code.hpp>

#ifndef _ZEABUS_ROS_INTERFACES_PATH_FILE_HPP__
#define _ZEABUS_ROS_INTERFACES_PATH_FILE_HPP__

namespace zeabus
{

namespace ros_interfaces
{

namespace file
{

    class PathFile
    {

        // We don't recommand setup full_path because we can't to check and alert you
        //  about your fullpat are exi
        PathFile( std::string full_path = "");

        bool setup_package( std::string package_name );
        void setup_subdirectory( std::string subdirectory );
        void setup_file_name( std::string file_name );

        // Return true not mean you success open file or file are exist
        // but if true mean we updated full path and you already setup 3 path
        bool updated_path();

        protected:
            std::string path_package;
            std::string path_subdirectory;
            std::string file_name;
            std::string full_path;

    }; // PathFile Object

} // namespace file

} // namespace ros_interfaces

} // namespace zeabus

#endif // _ZEABUS_ROS_INTERFACES_FILE_HPP__
