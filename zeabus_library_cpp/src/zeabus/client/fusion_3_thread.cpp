// FILE			: fusion_3_thread.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

#include    <zeabus/ros_interfaces/fusion_3_thread.hpp>

namespace zeabus
{

namespace client
{

    Fusion3Thread( std::shared_ptr< ros::NodeHandle > ptr_node_handle )
    {
        (this->pressure_client).setup_ptr_node_handle( ptr_node_handle );
        (this->dvl_client).setup_ptr_node_handle( ptr_node_handle );
        (this->imu_client).setup_ptr_node_handle( ptr_node_handle );
    } // function constructor Fusion3Thread

    void Fusion3Thread::setup_ptr_mutex_data( std::shared_ptr< std::mutex > dvl_mutex 
            , std::shared_ptr< std::mutex > imu_mutex 
            , std::shared_ptr< std::mutex > pressure_mutex )
    {
        (this->pressure_client).setup_ptr_mutex_data( dvl_mutex );
        (this->dvl_client).setup_ptr_mutex_data( imu_mutex );
        (this->imu_client).setup_ptr_mutex_data( pressure_mutex );
    } // setup_ptr_mutex_data

    void Fusion3Thread::setup_data( geometry_msgs::Vector3Stamped* dvl_data
            , sensor_msgs::Imu* imu_data , zeabus_utility::HeaderFloat64* pressure_data )
    {
        this->dvl_data = dvl_data;
        this->imu_data = imu_data;
        this->pressure_data = pressure_data;
    } // setup_data

    void Fusion3Thread::setup_client( std::string* dvl_topic , std::string* imu_topic
            , std::string* pressure_topic )
    {
        (this->dvl_client).setup_client( dvl_topic );
        (this->imu_client).setup_client( imu_topic );
        (this->pressure_client).setup_client( pressure_topic );
    } // setup_client function

    void Fusion3Thread::all_call()
    {

    } // all_call() function
} // namespace client

} // namespace zeabus
