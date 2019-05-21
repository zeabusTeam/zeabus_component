// FILE			: trimed_imu.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 20 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
//  _LOG_FILTER_    :   This will collect all log of filter
//  _COLLECT_LOG_   :   This will collect all log in this code
//  _LOG_IN_OUT_    :   This will collect all log about pair data of input and output

// README
//  IMU filter this find we will have to convert quaternion to roll pitch yaw because if I use
//  mean to avarage quaternion that may be affect to characteristic of quaternion about unit 
//  vector.

// REFERENCE

// MACRO SET
//#define _LOG_FILTER_
//#define _LOG_IN_OUT_
//#define _COLLECT_LOG_

// MACRO COMMAND
#ifdef _COLLECT_LOG_
    #define _LOG_FILTER_
    #define _LOG_IN_OUT_
#endif

#include    <iostream>

#include    <memory>

#include    <zeabus/ros_interfaces/singled_thread.hpp>

#include    <zeabus/client/single_thread/get_sensor_imu.hpp>

#include    <zeabus/service/get_single_data/sensor_imu.hpp>

#include    <geometry_msgs/sensor_imu.h>

#include    <zeabus/escape_code.hpp>

#include    <zeabus/time.hpp>

#include    <zeabus/count.hpp>

#ifdef _LOG_FILTER_
#include    <zeabus/ros_interfaces/file/vector3_filter.hpp>
#endif

#ifdef _LOG_IN_OUT_
#include    <zeabus/ros_interfaces/file/quaternion_filter.hpp>
#endif

// Part of algoritm
#include    <zeabus/filter/trimed_meani_two_pi.hpp>

int main( int argv , char** argc )
{
    // First part is about base file in ros system
    zeabus::ros_interfaces::SingleThread node_imu_filter( argv , argc , "filter_imu" );
    
    std::shared_ptr< ros::NodeHandle > ptr_node_handle = 
            std::make_shared< ros::NodeHandle >("");

    bool process_code = true;

    // This lock will use only about server because we don't know when have request result
    std::shared_ptr< std::mutex > ptr_mutex_data = std::make_shared< std::mutex >();

    // Insert optional part param
    const static unsigned int buffer_size = 6;
    const static unsigned int trimed_size = 1;
    const static unsigned int frequency = 50;
    const static unsigned int limit_same_time = 10;

    // Second part of Filter this part mix about data variable
    // template< size of buffeer , size of trimed >
    zeabus::filter::TrimedMean2Pi< buffer_size , trimed_size > roll_filter;
    zeabus::filter::TrimedMean2Pi< buffer_size , trimed_size > pitch_filter;
    zeabus::filter::TrimedMean2Pi< buffer_size , trimed_size > yaw_filter;
    
    // Third part shared variable in this case is variable for input and output
    sensor_msgs::Imu input_data;
    sensor_msgs::Imu output_data;
    output_data.header.frame_id = "filter/imu";
    ros::Time time_stamp = ros::Time::now();
    ros::Rate rate( frequency );

    // Forth part setup service
    zeabus::service::get_single_data::SensorImu server_imu_filter;
    server_imu_filter.setup_ptr_node_handle( ptr_node_handle );
    server_imu_filter.setup_ptr_mutex_data( ptr_mutex_data );
    server_imu_filter.register_data( &output_data );

    // Fifth part setup client
    zeabus::client::single_thread::GetSensorImu client_imu_sensor;
    client_imu_sensor.setup_ptr_node_handle( ptr_node_handle );;
    client_imu_sensor.setup_ptr_data( &input_data );
    // We don't setup mutex for this because we didn't use that

} // function main
