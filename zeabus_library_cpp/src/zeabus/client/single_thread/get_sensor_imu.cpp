// FILE         : get_sensor_imu.cpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

#include    <zeabus/client/single_thread/get_sensor_imu.hpp>

namespace zeabus
{

namespace client
{

namespace single_thread
{

    GetSensorImu::GetSensorImu( std::shared_ptr< ros::Nodehandle > ptr_node_handle )
        : BaseClass( ptr_node_handle )
    {
        this->save_time = ros::Time::now();
    } // constructor of GetSensorImu

    bool GetSensorImu::setup_client( std::string topic_service )
    {
        this->client_server = this->ptr_node_handle->serviceClient<zeabus_utility::GetSensorImu>(
                topic_service );
        return true;
    } // function setup_client

    bool GetSensorImu::normal_call( bool data )
    {
        (this->client_data).request.data = data;
        (this->client_server).call( this->client_data );
        *(this->ptr_data) = (this->client_data).response.data;
    } // function normal_call

    bool GetSensorImu::mutex_call( bool data )
    {
        (this->client_data).request.data = data;
        (this->client_server).call( this->client_data );
        this->ptr_mutex_data->lock()
        *(this->ptr_data) = (this->client_data).response.data;
        this->ptr_mutex_data->unlock()
        this->thread_status[0] = false; // finish run thread
    }

    bool GetSensorImu::thread_call( bool data )
    {
        bool run_thread = false;
        if( ! ( this->thread_status[0] ) )
        {
            this->thread_status[0] = true;
            this->thread_id[0] = std::thread( std::bind( 
                    &zeabus::client::single_thread::GetSensorImu::mutex_call , this )
                , data );
            run_thread = true;
        } // thread are not running
        return run_thread;
    }

} // namespace single_thread

} // namespace client

} // namespace zeabus

