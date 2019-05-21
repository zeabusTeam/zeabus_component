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

#include    <memory>

#include    <iostream>

#include    <zeabus/time.hpp>

#include    <zeabus/count.hpp>

#include    <zeabus/escape_code.hpp>

#include    <geometry_msgs/sensor_imu.h>

#include    <zeabus/ros_interfaces/singled_thread.hpp>

#include    <zeabus/client/single_thread/get_sensor_imu.hpp>

#include    <zeabus/service/get_single_data/sensor_imu.hpp>

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
    double input_filter[3];
    double output_filter[3];

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

    // Optional part about log if you want to do must define 
#ifdef _LOG_FILTER_
    zeabus::ros_interfaces::file::Vector3Filter file_filter;
    file_filter.setup_package( "zeabus_log" );
    file_filter.setup_directory("log/filter/imu");
    file_filter.setup_file_name("imu_trimed_mean" + zeabus::local_time( 6) + ".txt" );
    process_code = file_filter.open();
    if( !process_code )
    {
        std::cout   << zeabus::escape_code::normal_yellow << "WARNING! filter file can't open\n"
                    << zeabus::escape_code::normal_white;
    }
#endif // _LOG_FILTER_
#ifdef _LOG_IN_OUT_
    zeabus::ros_interfaces::file::QuaternionFilter file_in_out;
    file_in_out.setup_package("zeabus_log");
    file_in_out.setup_directory( "log/filter/imu" );
    file_in_out.setup_file_name( "imu_quaternion" + zeabus::local_time( 6 ) + ".txt" );
    process_code = file_in_out.open();
    if( !process_code )
    {
        std::cout   << zeabus::escape_code::normal_yellow << "WARNING! in-out file can't open\n"
                    << zeabus::escape_code::normal_white;
    }
#endif // _LOG_IN_OUT_

} // function main
