// FILE         : base_class.cpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 30
// MAINTAINER   : Supasan Komonlit

#include    <zeabus/service/base_class.hpp>

#ifndef _ZEABUS_SERVICE_BASE_CLASS_CPP__
#define _ZEABUS_SERVICE_BASE_CLASS_CPP__

namespace zeabus
{

namespace service
{

    BaseClass::BaseClass( std::shared_ptr< ros::NodeHandle > ptr_node_handle )
    {
        if( ptr_node_handle != NULL )
        {
            this->setup_ptr_node_handle( ptr_node_handle );
        }
        else
        {
            this->already_setup_ptr_node_handle = false;
        }
        this->already_setup_ptr_mutex_data = false;
    } // constructor BaseClass

    void BaseClass::setup_ptr_node_handle( std::shared_ptr< ros::NodeHandle > ptr_node_handle )
    {
        this->ptr_node_handle = ptr_node_handle;
        this->already_setup_ptr_node_handle = true;  
    }

    void BaseClass::setup_ptr_mutex_data( std::shared_ptr< ros::NodeHandle > ptr_mutex_data )
    {
        this->ptr_mutex_data = ptr_mutex_data;
        this->already_setup_ptr_mutex_data = true;
    }

} // namespace service

} // namespace zeabus

#endif // _ZEABUS_SERVICE_BASE_CLASS_HPP__
