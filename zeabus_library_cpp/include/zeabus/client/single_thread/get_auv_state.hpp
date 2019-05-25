// FILE			: get_auv_state.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 25 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <zeabus/client/single_thread/base_class.hpp>

#include    <zeabus_utility/AUVState.h>

#include    <zeabus_utility/GetAUVState.h>

#ifndef _ZEABUSE_CLIENT_SINGLE_THREAD_GET_AUV_STATE_HPP__
#define _ZEABUSE_CLIENT_SINGLE_THREAD_GET_AUV_STATE_HPP__

namespace zeabus
{

namespace client
{

namespace single_thread
{

    class GetAUVState : public BassClass< zeabus_utility::AUVState >
    {
        public:
            GetAUVState( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL );

            bool setup_client( std::string topic_service );
    
            bool normal_call()

        protected:
            ros::ServiceClient client_server;
            zeabus_utility::GetAUVState client_data;
    }; // class GetAUVState

} // single_thread

} // client

} // zeabus

#endif
