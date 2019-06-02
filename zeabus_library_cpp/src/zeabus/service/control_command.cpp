// FILE			: control_command.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 25 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MARCO CONDITION

#include    <zeabus/service/control_command.hpp>

namespace zeabus
{

namespace service
{

    ControlCommand::ControlCommand( std::shared_ptr< ros::NodeHandle > ptr_node_handle )
            : BaseClass( ptr_node_handle )
    {
        this->already_setup_ptr_data = false;
    } // function structure ControlCommand

    bool ControlCommand::setup_server_service( std::string service_topic )
    {
        bool result = true;
        if( ! (this->already_setup_ptr_data ) )
        {
            std::cout   << zeabus::escape_code::bold_red 
                        << "Service of control can't setup please setup_ptr_data\n"
                        << zeabus::escape_code::normal_white;
            result = false;
        }
        if( ! ( this->already_setup_ptr_mutex_data ) )
        {
            std::cout   << zeabus::escape_code::bold_red
                        << "Service of control can't setup please setup_ptr_mutex_data\n"
                        << zeabus::escape_code::normal_white;
            result = false;
        }
        if( ! ( this->already_setup_ptr_node_handle ) )
        {
            std::cout   << zeabus::escape_code::bold_red
                        << "Service of control can't setup please setup_ptr_node_handle\n"
                        << zeabus::escape_code::normal_white;
            result = false;
        }
        if( result )
        {
            this->server_service = this->ptr_node_handle->advertiseService( service_topic 
                , &zeabus::service::ControlCommand::callback , this );
        }
        return result;
    } // function setup_server_service
    
    bool ControlCommand::setup_ptr_data( zeabus_utility::ControlCommand* ptr_control_command )
    {
        this->ptr_control_command = ptr_control_command;
        this->already_setup_ptr_data = true;
        return true;
    } // function setup_ptr_data

    bool ControlCommand::callback( zeabus_utility::SendControlCommand::Request& request
            , zeabus_utility::SendControlCommand::Response& response )
    {
        this->ptr_mutex_data->lock();
        *(this->ptr_control_command) = request.command;
        this->ptr_mutex_data->unlock();
        return true;
    } // function callback


} // namespace service

} // namespace zeabus
