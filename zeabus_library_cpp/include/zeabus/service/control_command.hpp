// FILE			: control_command.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 24 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
//  ref1    : http://wiki.ros.org/msg

// MACRO SET

// MARCO CONDITION

#include    <zeabus/service/base_class.hpp>

#include    <zeabus_utility/SendControlCommand.h>

#ifndef _ZEABUS_SERVICE_CONTROL_COMMAND_HPP__
#define _ZEABUS_SERVICE_CONTROL_COMMAND_HPP__

namespace zeabus
{

namespace service
{

    class ControlCommand : public zeabus::service::BaseClass
    {
        public:
            ControlCommand( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL);
        
            bool setup_server_service( std::string service_topic );

            bool setup_ptr_data( zeabus_utility::ControlCommand* ptr_control_command );

        protected:
            bool callback( zeabus_utility::SendControlCommand::Request& request 
                    , zeabus_utility::SendControlCommand::Response& response );

            zeabus_utility::ControlCommand* ptr_control_command;

            bool already_setup_ptr_data;

            ros::ServiceServer server_service;


        
    }; // object ControlCommand

} // namespace service

} // namespace zeabus

#endif
