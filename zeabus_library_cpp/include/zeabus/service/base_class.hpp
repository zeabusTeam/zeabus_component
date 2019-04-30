// FILE         : base_class.hpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 30
// MAINTAINER   : Supasan Komonlit

#include    <memory>

#include    <iostream>

#include    <ros/ros.h>

#include    <mutex>

#ifndef _ZEABUS_SERVICE_BASE_CLASS_HPP__
#define _ZEABUS_SERVICE_BASE_CLASS_HPP__

namespace zeabus
{

namespace service
{

    class BaseClass
    {
        public:
            BaseClass( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL );

            void setup_ptr_node_handle( std::shared_ptr< ros::NodeHandle > ptr_node_handle );

            void setup_ptr_mutex_data( std::shared_ptr< std::mutex > ptr_mutex_data );

            virtual bool setup_server_sevice( std::string service_topic ) = 0 ;

        protected:
            std::shared_ptr< ros::NodeHandle > ptr_node_handle;
            std::shared_ptr< std::mutex > ptr_mutex_data;

            bool already_setup_ptr_node_handle;
            bool already_setup_ptr_mutex_data;

    }; // BaseClass object

} // namespace service

} // namespace zeabus

#endif // _ZEABUS_SERVICE_BASE_CLASS_HPP__
