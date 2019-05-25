// FILE			: get_auv_state.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 25 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include <zeabus/client/single_thread/get_auv_state.hpp>

namespace zeabus
{

namespace client
{

namespace single_thread
{

    GetAUVState::GetAUVState( std::shared< ros::NodeHandle > ptr_node_handle )
            : BaseClass( ptr_node_handle )
    {
        ;
    } // constructor of GetAUVState

    bool GetAUVState::setup_client( std::string topic_service )
    {
        bool result = false;
        if( this->already_setup_ptr_node_handle )
        {
            this->client_server = this->ptr_node_handle->serviceClient< 
                    zeabus_utility::GetAUVState >( topic_service );
            result = true;
        }
    }

    bool GetAUVState::normal_call()
    {

    }

} // namespace single_thread

} // namespace client

} // namespace zeabus
