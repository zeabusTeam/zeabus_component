// FILE         : base_class.hpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

#include    <ros/ros.h>

#include    <iostream>

#include    <memory>

#include    <thread>

//  MACRO DETAIL
//  _PRINT_JOIN_PROCESS_    : will print after join thread finish


// README
// This base class will help you to manage about thread.
// Focuse on thread for call service 
// Subclass will use this to active all call and user can use this join to join all thread/
// This BaseClass will request only NodeHandle for pattern in ros system
// Why this don't have mutex or data because will don't know what mutex or what data you have.

#ifndef _ZEABUS_CLIENT_BASE_CLASS_HPP__
#define _ZEABUS_CLIENT_BASE_CLASS_HPP__

namespace zeabus
{

namespace client
{

    template< unsigned int size_thread >
    class BaseClass
    {
        public:
            BaseClass( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL );

            void setup_ptr_node_handle( std::shared_ptr< ros::NodeHandle > ptr_node_handle );

            void thread_join();

            virtual void thread_call() = 0;

        protected:
            std::shared_ptr< ros::NodeHandle > ptr_node_handle;
            std::thread thread_id[ size_thread ];

    }; // class BaseClass
 
} // namespace client
   
} // namespace zeabus

#include    <zeabus/client/base_class.cpp>

#endif // _ZEABUS_CLIENT_BASE_CLASS_HPP__
