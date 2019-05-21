// FILE         : get_geometry_vector3_stamped.cpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, MAY 10
// MAINTAINER   : K.Supasan

#include    <zeabus/client/single_thread/get_geometry_vector3_stamped.hpp>

namespace zeabus
{

namespace client
{

namespace single_thread
{

    GetGeometryVector3Stamped::GetGeometryVector3Stamped( std::shared_ptr< ros::NodeHandle > 
            ptr_node_handle ) : BaseClass( ptr_node_handle )
    {
        ;
    } // constructor GetGeometryVector3Stamped

    bool GetGeometryVector3Stamped::setup_client( std::string topic_service )
    {
        if( this->already_setup_ptr_node_handle )
        {
            this->client_server = this->ptr_node_handle->serviceClient< 
                    zeabus_utility::GetGeometryVector3Stamped >( topic_service ); 
        }
        else
        {
            std::cout   << zeabus::escape_code::bold_red << "Please setup ptr_node_handle\n"
                        << zeabus::escape_code::normal_white;
        }
        return this->already_setup_ptr_node_handle;
    } // function setup_client

    bool GetGeometryVector3Stamped::normal_call()
    {
        bool result = ! ( this->thread_status[0] );
        if( result )
        {
            (this->client_server).call( this->client_data );
            *(this->ptr_data) = (this->client_data).response.data;
        }
        return result;
    }

    bool GetGeometryVector3Stamped::thread_call()
    {
        bool can_call = ! ( this->thread_status[0] );
        if( can_call )
        {
            (this->thread_status)[0] = true;
            (this->thread_id)[0] = std::thread( 
                    &zeabus::client::single_thread::GetGeometryVector3Stamped::mutex_call, this);
        }
        return can_call;
    }

    void GetGeometryVector3Stamped::mutex_call()
    {
        (this->thread_status)[0] = true;
        (this->client_server).call( this->client_data );
        this->ptr_mutex_data->lock();
        *(this->ptr_data) = (this->client_data).response.data;
        this->ptr_mutex_data->unlock();
    }

} // namespace single_thread

} // namespace client

} // namespace zeabus

