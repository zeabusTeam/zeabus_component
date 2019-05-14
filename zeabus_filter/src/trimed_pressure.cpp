// FILE			: trimed_pressure.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 13 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
//  _DEBUG_FILTER_  : This will print about input and output of filter
//  _DEBUG_SPIN_    : This will print about spin new thread and will check before full active

// README
//  This file use trimed_mean method to filter value from pressure sensor
//  This file will have 2 side first client for request data for pressure sensor node
//     second is server for send result of filter

// REFERENCE
//  ref1    : http://docs.ros.org/melodic/api/roscpp/html/classros_1_1NodeHandle.html#ac304301e255efeb8c34addfe7fa79b49
//  ref2    : http://docs.ros.org/melodic/api/roscpp/html/classros_1_1NodeHandle.html#a81d08224944993c4cd38589e68417e12

// MACRO SET
#define _DEBUG_FILTER_
#define _DEBUG_SPIN_

// Header of genral part that mean you can change algorithm to fileter but can't change this part

#include    <iostream>

#include    <zeabus/ros_interfaces/single_thread.hpp>

#include    <zeabus/client/single_thread/get_depth_command.hpp>

#include    <zeabus/service/get_single_data/header_float64.hpp>

#include    <zeabus_utility/HeaderFloat64.h>

#include    <zeabus/escape_code.hpp>

#include    <zeabus/time.hpp>

#include    <zeabus/ros_interfaces/file/single_filter.hpp>

// Part of algorithm
#include    <zeabus/filter/trimed_mean.hpp>

// Below function I will test about time up of data
bool check_time_up( unsigned int limit_count , bool not_equal );

int main( int argv, char** argc )
{
    //  First part is about base file in ros system have 3 sub-part
    //  --->First sub-part is init_node please init by ros_intefaces we implement for many thread
    //  --->Second sub-part is about ptr_node_handle about my class to help manage client
    //  ------->and service must use ptr_node_handle to active
    //  --->Third sub-part is mutex to use for lock data between thread 
    //  ------->Warning not between process  
    zeabus::ros_interfaces::SingleThread node_pressure_filter( argv , argc , "filter_pressure" );

    std::shared_ptr< ros::NodeHandle > ptr_node_handle = 
            std::make_shared< ros::NodeHandle >("");

    bool process_code = true; // this use to collect process of code

    // This lock will use only about server because we don't know when have request result
    std::shared_ptr< std::mutex > ptr_mutex_data = std::make_shared< std::mutex >();

    // Insert optional part param part  
    const static unsigned int buffer_size = 10; // size of buffer to use collect data
    const static unsigned int trimed_size = 2; // size of buffer will trimed
    const static unsigned int frequency = 60; // frequency of client to request data for sensor
    const static unsigned int limit_same_time = 10; // limit to warning when receive 10 time

    // Second part of Filter this part mix about data variable
    // template< type buffer , size of buffer >
    zeabus::filter::TrimedMean< double , buffer_size > filter_pressure( trimed_size );

    // Third part shared variable in this case is variable for input and output
    zeabus_utility::HeaderFloat64 input_data;
    zeabus_utility::HeaderFloat64 output_data;
    output_data.header.frame_id = "filter/pressure";
    ros::Time time_stamp = ros::Time::now(); // Time stamp for check new or old data
    ros::Rate rate( frequency ); 
    
    // Forth part setup service
    zeabus::service::get_single_data::HeaderFloat64 server_pressure_filter;
    server_pressure_filter.setup_ptr_node_handle( ptr_node_handle );
    server_pressure_filter.setup_ptr_mutex_data( ptr_mutex_data );
    server_pressure_filter.register_data( &output_data );
    // We don't want to setup server now because our data don't already
//    process_code = server_pressure_filter.setup_server_service( "/filter/pressure" );
    if( ! process_code )
    {
        std::cout   << zeabus::escape_code::bold_red << "Filter pressure can't setup server\n"
                    << zeabus::escape_code::normal_white;
    }
    
    // Fifth part setup client
    zeabus::client::single_thread::GetDepthCommand client_pressure_sensor;
    client_pressure_sensor.setup_ptr_node_handle( ptr_node_handle );
    client_pressure_sensor.setup_ptr_data( &input_data );
    // We don't setup mutex for this because we didn't use that

    // Sixth part full filter part buffer
    for( unsigned int run = 0 ; run < buffer_size ; run++ )
    {
        rate.sleep();
        (void)client_pressure_sensor.normal_call();
        if( check_time_up( limit_same_time , time_stamp != input_data.header.stamp ) )
        {
            time_stamp = input_data.header.stamp;
            output_data.data = filter_pressure.push( input_data.data );
            output_data.header.stamp = ros::Time::now();
#ifdef _DEBUG_FILTER_
            std::cout   << "First fill push " << input_data.data
                        << " and result of fileter " << output_data.data;
#endif 
        }
    } // for loop for fill buffer first time

    // Seven part let start ros loop
    // Now I already data for output I will setup server
    process_code = server_pressure_filter.setup_server_service( "/filter/pressure" );
    if( ! process_code )
    {
        std::cout   << zeabus::escape_code::bold_red << "Filter pressure can't setup server\n"
                    << zeabus::escape_code::normal_white;
        ptr_node_handle->shutdown();
    }

    process_code = node_pressure_filter.spin(); // This command will split thread to spin
    if( ! process_code ) //  will fals if we can't success to create new thread 
    {
        std::cout   << zeabus::escape_code::bold_red << "Please find why we can creath thread\n"
                    << zeabus::escape_code::normal_white;
        ptr_node_handle->shutdown();
    }
    else
    {
#ifdef _DEBUG_SPIN_
        std::cout   << "node_pressure_filter waiting to sure we are spining\n";
#endif // _DEBUG_SPIN_
        while( ! ( node_pressure_filter.status() ) ) //  we want to ensure now I spin
        {
            rate.sleep();
        }
#ifdef _DEBUG_SPIN_
        std::cout   << "node_pressure_filter finish spin\n";
#endif // _DEBUG_SPIN_
    } // condition fo spin by new thread

    
    while( ptr_node_handle->ok() )
    {
        rate.sleep();
        (void)client_pressure_sensor.normal_call();
        if( check_time_up( limit_same_time , time_stamp != input_data.header.stamp ) )
        {
            time_stamp = input_data.header.stamp;
            (void)filter_pressure.push( input_data.data );
            ptr_mutex_data->lock();
            output_data.header.stamp = ros::Time::now();
            output_data.data = filter_pressure.get_result();
            ptr_mutex_data->unlock();
#ifdef _DEBUG_FILTER_
            std::cout   << "ros loop push " << input_data.data
                        << " and result of fileter " << output_data.data;
#endif // _DEBUG_FILTER_
        }
    }

    // Next part is last part we have to close all thread and all ros operate by this code
    ros::shutdown(); // we want to close all NodeHandle in this pid
    node_pressure_filter.join();

    return 0;
     
}

// t
bool check_time_up( unsigned int limit_count , bool not_equal )
{
    static unsigned int count = 1;
    if( ! not_equal )
    {
        if( count == limit_count )
        {
            std::cout   << zeabus::escape_code::bold_yellow << "Warning data didn't updated\n"
                        << zeabus::escape_code::normal_white;
        }
        else
        {
            count++;
        } 
    }
    else
    {
        
        count = 1;
    }
    return not_equal;
}
