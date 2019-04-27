// FILE         : base_class.hpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 28
// MAINTAINER   : Supasan Komonlit

#include    <memory> // use smart_pointer

#include    <iostream> // include standard library of cpp

#include    <ros/ros.h> // include library about ros for CXX language  

#ifndef _ZEABUS_SERVICE_GET_SINGLE_DATA_BASE_CLASS_HPP__
#define _ZEABUS_SERVICE_GET_SINGLE_DATA_BASE_CLASS_HPP__

namespace zeabus
{

namespace service
{

namespace get_single_data
{

    template< class data_type >
    class BaseClass
    {
        public:
            BaseClass( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL );

            void setup_ptr_node_handle( std::shared_ptr< ros::NodeHandle > ptr_node_handle );

            void register_data( data_type* ptr_data );

            bool setup_server_service( std::string service_topic );

            void check_setup_service();

            ros::ServiceServer service_server;

        protected:

            virtual void ensure_setup_service( std::string service_topic ) = 0;

            std::shared_ptr< ros::NodeHnalde > ptr_node_handle;

            data_type* ptr_message;

            bool already_setup_ptr_data;

            bool already_setup_ptr_node_handle;

    }; // BaseClass object

} // name get_single_data

} // namespace service
 
} // namespace zeabus

#include    <zeabus/service/get_single_data/base_class.cpp>

#endif // _ZEABUS_SERVICE_GET_SINGLE_DATA_BASE_CLASS_HPP__
