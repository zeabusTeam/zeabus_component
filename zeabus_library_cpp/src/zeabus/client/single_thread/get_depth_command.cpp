// FILE         : get_depth_command.cpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 9
// MAINTAINER   : K.Supasan

#include    <zeabus/client/single_thread/get_depth_command.hpp>

namespace zeabus
{

namespace client
{

namespace single_thread
{

    GetDepthCommand::GetDepthCommand( std::shared_ptr< ros::NodeHandle > ptr_node_handle )
            : BaseClass( ptr_node_handle ) 
    {
        ;
    } // constructor object

    bool GetDepthCommand::setup_client( std::string topic_service )
    {
        bool result = false;
        if( this->already_setup_ptr_node_handle )
        {
            this->client_server = this->ptr_node_handle->serviceClient<
                    zeabus_utility::DepthCommand>( topic_service ); 
            result = true;
        }
        else
        {
            std::cout   << zeabus::escape_code::bold_red << "Please setup ptr_node_handle!!\n"
                        << zeabus::escape_code::normal_white;
        }
        return result;
    }

    bool GetDepthCommand::normal_call()
    {
        bool result = ! (this->thread_status)[0];
        if( result )
        {
            (this->client_server).call( this->client_data );
            this->ptr_data->header = (this->client_data).response.header;
            this->ptr_data->data = (this->client_data).response.depth;
        }
        return result;
    };

    void GetDepthCommand::mutex_call()
    {
        (this->client_server).call( this->client_data );
        this->ptr_mutex_data->lock();
        this->ptr_data->header = (this->client_data).response.header;
        this->ptr_data->data = (this->client_data).response.depth;
        this->ptr_mutex_data->unlock();
        (this->thread_status)[0] = false;
    }

    bool GetDepthCommand::thread_call()
    {
        bool result = !( this->thread_status)[0];
        if( result )
        {
            (this->thread_status)[0] = true; // Mark thread will run in the next time
            (this->thread_id)[0] = std::thread( 
                    &zeabus::client::single_thread::GetDepthCommand::mutex_call , this );
        }
        return result;
    }

} // namespace single_thread

} // namespace client

} // namespace zeabus
