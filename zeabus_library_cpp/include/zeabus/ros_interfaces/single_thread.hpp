// FILE         : single_thread.hpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 28
// MAINTAINER   : Supasan Komonlit

#include    <iostream>

#include    <ros/ros.h>

#include    <thread>

#include    <zeabus/escape_code.hpp>

#include    <mutex>

#ifndef _ZEABUS_ROS_INTERFACES_SINGLE_THREAD_HPP__
#define _ZEABUS_ROS_INTERFACES_SINGLE_THREAD_HPP__

namespace zeabus
{

namespace ros_interfaces
{

    class SingleThread 
    {
        public :
            SingleThread( int argv , char** argc , std::string node_name );

            ~SingleThread();

            // User use this function to spin thread
            // If return true success spin in otherwise false
            bool spin();

            bool status();

            void join();

        protected:
            // Use this to spilt thread processs
            void thread_spin();

            bool status_thread;

            std::string node_name;

            std::mutex mutex_lock;

            std::thread thread_id;

    }; // namespace SingleThread

} // namespace ros_interfaces

} // namespace zeabus

#endif // _ZEABUS_ROS_INTERFACES_SINGLE_THREAD_
