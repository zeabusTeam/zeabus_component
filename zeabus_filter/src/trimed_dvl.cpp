// FILE			: trimed_dvl.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
// _COLLECT_LOG_    : This will help you to collect log of your data
// _SUMMARY_        : This will help you show only important data

// README
//  In dvl and pressure sensor will use only one log because input and output is same variable

// REFERENCE

// MACRO SET
#define _COLLECT_LOG_
//#define _SUMMARY_

#include    <memory>

#include    <iostream>

#include    <zeabus/time.hpp>

#include    <zeabus/count.hpp>

#include    <zeabus/escape_code.hpp>

#include    <zeabus/filter/trimed_mean.hpp>

#include    <geometry_msgs/Vector3Stamped.h>

#include    <zeabus/ros_interfaces/single_thread.hpp>

#include    <zeabus/ros_interfaces/file/vector3_filter.hpp>

#include    <zeabus/service/get_data/geometry_vector3_stamped.hpp>

#include    <zeabus/client/single_thread/get_geometry_vector3_stamped.hpp>

int main( int argv , char** argc )
{
    // First part is about base file in ros system
    zeabus::ros_interfaces::SingleThread node_dvl_filter( argv , argc , "filter_dvl" );
    
    std::shared_ptr< ros::NodeHandle > ptr_node_handle = 
            std::make_shared< ros::NodeHandle >("");

    std::shared_ptr< std::mutex > ptr_mutex_data = std::make_shared< std::mutex >();

    // Inset optional part param part
    const static unsigned int buffer_size = 6;
    const static unsigned int trimed_size = 1;
    const static unsigned int frequency = 50;
    const static unsigned int limit_same_time = 5;
    static bool time_over;

    // Second part of Filter this partmix about data variable
    zeabus::filter::TrimedMean< double, buffer_size > filter_dvl[3];
    filter_dvl[0].setup_trim( trimed_size );
    filter_dvl[1].setup_trim( trimed_size );
    filter_dvl[2].setup_trim( trimed_size );

    // Third part shared variable in this case is variable for input and output
    geometry_msgs::Vector3Stamped input_data;
    geometry_msgs::Vector3Stamped output_data;
    output_data.header.frame_id = "base_dvl";
    ros::Time time_stamp = ros::Time::now();
    ros::Rate rate( frequency );

    // Forth part setup service
    zeabus::service::get_data::GeometryVector3Stamped server_dvl_filter;
    server_dvl_filter.setup_ptr_node_handle( ptr_node_handle );
    server_dvl_filter.setup_ptr_mutex_data( ptr_mutex_data );
    server_dvl_filter.register_data( &output_data );

    // Fifth part setup client
    zeabus::client::single_thread::GetGeometryVector3Stamped client_dvl_filter;
    client_dvl_filter.setup_ptr_node_handle( ptr_node_handle );
    client_dvl_filter.setup_ptr_data( &input_data );
    client_dvl_filter.setup_client( "/sensor/dvl" );

    // Option part about log file ig you want to do must define _COLLECT_LOG_
#ifdef _COLLECT_LOG_
    zeabus::ros_interfaces::file::Vector3Filter my_file;
    my_file.setup_package( "zeabus_log" );
    my_file.setup_subdirectory( "log/filter/dvl" );
    my_file.setup_file_name( "dvl_trimed_mean" + zeabus::local_time( 6 ) + ".txt" );
    (void)my_file.open();
    my_file.write_column();
#endif


// This part will full buffer
    for( unsigned int run = 0 ; run < buffer_size ; run++ )
    {
        rate.sleep();
        (void)client_dvl_filter.normal_call();
        if( zeabus::count::compare( input_data.header.stamp , limit_same_time , &time_over ) )
        {
            time_stamp = input_data.header.stamp;
            (void)filter_dvl[0].push( input_data.vector.x );
            (void)filter_dvl[1].push( input_data.vector.y );
            (void)filter_dvl[2].push( input_data.vector.z );
            output_data.vector.x = filter_dvl[0].get_result();
            output_data.vector.y = filter_dvl[1].get_result();
            output_data.vector.z = filter_dvl[2].get_result();
            output_data.header.stamp = time_stamp;
#ifdef _COLLECT_LOG_
            my_file.logging( &time_stamp , &(input_data.vector) , &(output_data.vector) );
#endif // _COLLECT_LOG_
        }
        else if( time_over )
        {
            std::cout   << zeabus::escape_code::bold_red << "Fatal same stamp time of DVL"
                        << zeabus::escape_code::normal_white;
        }
        else
        {
            ;
        }
    }

    if( ! (server_dvl_filter.setup_server_service( "/filter/dvl" ) ) )
    {
        std::cout   << zeabus::escape_code::bold_red << "Filter dvl can't setup server\n"
                    << zeabus::escape_code::normal_white;
        ptr_node_handle->shutdown();
    }

    if( node_dvl_filter.spin()  )
    {
        while( ! (node_dvl_filter.status() ) )
        {
            rate.sleep();
        }
    }
    else
    {
        std::cout   << zeabus::escape_code::bold_red << "Please find why we can\'t create thread"
                    << zeabus::escape_code::normal_white << "\n";
        ptr_node_handle->shutdown();
    }

    while( ptr_node_handle->ok() )
    {
        rate.sleep();
        (void)client_dvl_filter.normal_call();
        if( zeabus::count::compare( input_data.header.stamp , limit_same_time , &time_over ) )
        {
            time_stamp = input_data.header.stamp;
            (void)filter_dvl[0].push( input_data.vector.x );
            (void)filter_dvl[1].push( input_data.vector.y );
            (void)filter_dvl[2].push( input_data.vector.z );
            ptr_mutex_data->lock();
            output_data.vector.x = filter_dvl[0].get_result();
            output_data.vector.y = filter_dvl[1].get_result();
            output_data.vector.z = filter_dvl[2].get_result();
            output_data.header.stamp = time_stamp;
            ptr_mutex_data->unlock();
#ifdef _COLLECT_LOG_
            my_file.logging( &time_stamp , &(input_data.vector) , &(output_data.vector) );
#endif // _COLLECT_LOG_
#ifdef _SUMMARY_
            zeabus::escape_code::clear_screen();
            std::cout   << "Input data vector : " << input_data.vector.x << " " 
                        << input_data.vector.y << " " << input_data.vector.z
                        << "\nOutput data vector: " << output_data.vector.x << " "
                        << output_data.vector.y << " " << output_data.vector.z << "\n";
#endif
        }
        else if( time_over )
        {
            std::cout   << zeabus::escape_code::bold_red << "FATAL DVL don't send new data\n"
                        << zeabus::escape_code::normal_white;
        }
        else
        {
            ;
        }
    }

    ros::shutdown();
    node_dvl_filter.join();
#ifdef _COLLECT_LOG_
    my_file.close();
#endif

    return 0;
}
