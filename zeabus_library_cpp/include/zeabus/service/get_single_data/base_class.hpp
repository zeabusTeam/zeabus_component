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

    template< class message_type >
    class BaseClass
    {
        public:
            BaseClass( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL );

            void setup_ptr_node_handle( std::shared_ptr< ros::NodeHandle > ptr_node_handle );

            void register_message( message_type* message );

            bool setup_server_service( std::string service_topic );

        protected:

            virtual void ensure_setup_service( std::string service_topic ) = 0;

            std::shared_ptr< ros::NodeHnalde > ptr_node_handle;

            bool can_setup_service;
        
    }; // BaseClass object

} // name get_single_data

} // namespace service
 
} // namespace zeabus


#endif // _ZEABUS_SERVICE_GET_SINGLE_DATA_BASE_CLASS_HPP__
