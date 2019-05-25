// FILE			: send_control_command.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 25 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MARCO CONDITION

#include    <zeabus/client/single_thread/base_class.hpp>

#include    <zeabus_utility/ControlCommand.h>

#include    <zeabus_utility/SendControlCommand.h>

#ifndef _ZEABUS_CLIENT_SINGLE_THREAD_SEND_CONTROL_COMMAND_HPP__
#define _ZEABUS_CLIENT_SINGLE_THREAD_SEND_CONTROL_COMMAND_HPP__

namespace zeabus
{

namespace client
{

namespace single_thread
{

    class SendControlCommand 
            : public zeabus::client::single_thread::BaseClass< zeabus_utility::ControlCommand > 
    {
        public:
            SendControlCommand( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL );

            bool setup_client( std::string topic_service );

            bool normal_call();
        
            bool thread_call();

            void mutex_call();

        protected:
            ros::ServiceClient client_service;
            zeabus_utility::SendControlCommand client_data;
    }; // class SendControlCommand

} // namespace single_thread

} // namespace client

} // namespace zeabus

#endif // _ZEABUS_CLIENT_SINGLE_THREAD_SEND_CONTROL_COMMAND_HPP__
