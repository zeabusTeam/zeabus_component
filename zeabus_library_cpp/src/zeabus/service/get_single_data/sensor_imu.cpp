// FILE         : sensor_imu.cpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 27
// MAINTAINER   : Supasan Komonlit

#include    <zeabus/service/get_single_data/sensor_imu.hpp>

namespace zeabus
{

namespace service
{

namespace get_single_data
{

    SensorImu::SensorImu( std::shared_ptr< ros::NodeHandle > ptr_node_handle 
            , std::string frame_id) : BaseClass( ptr_node_handle , frame_id )
    {
        ; // nothing to do in constructor fo subclass
    } //  Constructor SensorImu

    void SensorImu::ensure_setup_service( std::string service_topic )
    {
        this->service_server = this->ptr_node_handle->advertiseService( service_topic 
                , &zeabus::service::get_single_data::SensorImu::callback , this );
    } // function ensure_setup_service

    void SensorImu::setup_frame_id( std::string frame_id )
    {
        this->frame_id = frame_id;
    } // function setup_frame_id

    bool SensorImu::callback( zeabus_utility::GetSensorImu::Request& request 
            , zeabus_utility::GetSensorImu::Response& response )
    {
        response.data = *(this->ptr_data);
        response.header.stamp = ros::Time();
        response.header.frame_id = this->frame_id; 
        return true;
    } // function callback

} // namespace get_single_data

} // namespace service

} // namespace zeabus
