// FILE         : get_depth_command.hpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 9
// MAINTAINER   : K.Supasan

// MACRO DETAIL

// README
//  This file when connect with variable in main code use zeabus_utitlity/HeaderFloat64
//  But in class object use zeabus_utility/DepthCommand to connect with other node

#include    <zeabus/client/single_thread/base_class.hpp>

#include    <zeabus_utility/ServiceDepth.h>

#include    <zeabus_utility/HeaderFloat64.h>

#ifndef _ZEABUS_CLIENT_SINGLE_THREAD_GET_DEPTH_COMMAND_HPP__
#define _ZEABUS_CLIENT_SINGLE_THREAD_GET_DEPTH_COMMAND_HPP__

namespace zeabus
{

namespace client
{

namespace single_thread
{

    class GetDepthCommand 
            : public zeabus::client::single_thread::BaseClass< zeabus_utility::HeaderFloat64 >
    {
        public:
            GetDepthCommand( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL );

            bool setup_client( std::string topic_service );

            bool normal_call();

            bool thread_call();

            void mutex_call();

        protected:
            ros::ServiceClient client_server;
            zeabus_utility::ServiceDepth client_data;
    }; // class GetDepthCommand

} // namespace single_thread

} // namespace client

} // namespace zeabus

#endif // _ZEABUS_CLIENT_SINGLE_THREAD_GET_DEPTH_COMMAND_HPP__
