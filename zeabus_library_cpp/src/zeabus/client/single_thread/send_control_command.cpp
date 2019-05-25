// FILE			: send_control_command.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 25 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MARCO CONDITION

#include    <zeabus/client/single_thread/send_control_command.hpp>

namespace zeabus
{

namespace client
{

namespace single_thread
{

    SendControlCommand::SendControlCommand( std::shared_ptr< ros::NodeHandle > ptr_node_handle )
            : BaseClass( ptr_node_handle )
    {
        ;
    } // constructor SendControlCommand

    bool SendControlCommand::setup_client( std::string topic_service )
    {
        bool result = false;
        if( this->already_setup_ptr_node_handle )
        {
            this->client_service = this->ptr_node_handle->serviceClient< 
                    zeabus_utility::SendControlCommand >( topic_service );
            result = true;
        }
        return result;
    } // function SendControlCommand

    bool SendControlCommand::normal_call()
    {
        bool result = !(this->thread_status)[0];
        if( result )
        {
            this->client_data.request.command = *(this->ptr_data);
            (this->client_service).call( this->client_data );
        }
        return result;
    }

    bool SendControlCommand::thread_call()
    {
        bool result = !(this->thread_status)[0];
        if( result )
        {
            (this->thread_status)[0] = true;
            this->ptr_mutex_data->lock();
            (this->client_data).request.command = *(this->ptr_data);
            this->ptr_mutex_data->unlock();
            (this->thread_id)[0] = std::thread(
                    &zeabus::client::single_thread::SendControlCommand::mutex_call , this );
       }
        return result;
    }

    void SendControlCommand::mutex_call()
    {
        (this->client_service).call( this->client_data );
        (this->thread_status)[0] = false;
    }

} // namespace single_thread

} // namespace client

} // namespace zeabus
