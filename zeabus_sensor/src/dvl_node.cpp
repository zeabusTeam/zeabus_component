// File         : dvl_node.cpp
// Author       : Supasan Komonlit
// Create On    : 2019, APRIL 06

// MACRO DETAIL
// _SUMMARY_    : This macro will help you to see summary data

// REFERENCE
//  ref01   : http://wiki.ros.org/rosconsole
//  ref02   : http://docs.ros.org/melodic/api/roscpp/html/namespaceros_1_1param.html#a78f0407e63d1387348eb81ca47d71d79
//  ref03   : http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters

// MACRO SET
//#define _SUMMARY_

// MACRO CONDITION

#include    <memory>

#include    <thread>

#include    <iostream>

#include    <ros/ros.h>

#include    <ros/console.h>

#include    <geometry_msgs/Vector3Stamped.h>

#include    <zeabus/escape_code.hpp>

#include    <zeabus/sensor/DVL/connector.hpp>

#include    <zeabus/sensor/DVL/decode_string.hpp>

#include    <zeabus/ros_interfaces/single_thread.hpp>

#include    <zeabus/service/get_data/geometry_vector3_stamped.hpp>

namespace Asio = boost::asio;

void helper_status( bool data );

int main( int argv , char** argc )
{

    zeabus::ros_interfaces::SingleThread dvl_node( argv , argc , "dvl_node" );

    ros::NodeHandle param_handle("~");

    std::string full_path_port; 
    param_handle.param< std::string >( "full_path_port" 
        , full_path_port 
        , "/dev/usb2serial/ftdi_FT2VR5PM_02");

    std::string max_depth;
    param_handle.param< std::string >( "max_depth" , max_depth , "0060" );

    std::string heading;
    param_handle.param< std::string >( "heading" , heading , "09000" );

    std::string salinty;
    param_handle.param< std::string >( "salinty" , salinty , "35" );

    zeabus::sensor::DVL::Connector dvl( full_path_port );

    std::shared_ptr< ros::NodeHandle > ptr_node_handle = 
            std::make_shared< ros::NodeHandle >("");

    std::shared_ptr< std::mutex > ptr_mutex_data = std::make_shared< std::mutex >();

	ros::Publisher dvl_publisher = 
			ptr_node_handle->advertise<geometry_msgs::Vector3Stamped>("/sensor/dvl" , 1);

    if( ! dvl.open_port() )
    {
        ROS_FATAL_NAMED( "SENSOR_DVL" , "Failure to open port %s" , full_path_port.c_str() );
        ros::shutdown();
    }

    if( ptr_node_handle->ok() )
    {
	    (void)dvl.set_option_port( Asio::serial_port_base::flow_control( 
	            Asio::serial_port_base::flow_control::none ) );
	    (void)dvl.set_option_port( Asio::serial_port_base::parity( 
				Asio::serial_port_base::parity::none ) );
	    (void)dvl.set_option_port( Asio::serial_port_base::stop_bits( 
				Asio::serial_port_base::stop_bits::one ) );
	    (void)dvl.set_option_port( Asio::serial_port_base::character_size( (unsigned char) 8 ) );
	    (void)dvl.set_option_port( Asio::serial_port_base::baud_rate( 115200 ) );
        // idle in importance process because if can help guaruntee success setup
        if( !dvl.set_idle( 5 ) )
        {
            ROS_FATAL_NAMED( "SENSOR_DVL" , "Failure to open port");
            ros::shutdown();
        }
    }

    if( ptr_node_handle->ok() ) // open port is success
    {
        (void)dvl.load_parameter();
        (void)dvl.bottom_track( "001" );
        (void)dvl.max_depth( max_depth ); // 5 meter + 1 meter = 6 meter = 60 decimeter
        (void)dvl.set_heading( heading );
        (void)dvl.set_salinty( salinty );
        (void)dvl.time_per_ensemble( "00:00:00.00" );
        (void)dvl.time_between_pings( "00:00.05" );
        (void)dvl.ping_per_ensemble( "00000" );
        (void)dvl.data_stream( "6" );
        (void)dvl.save_parameter();
        (void)dvl.resume();
    }

    std::string raw_data; // collect data line from port
    std::string type_line; // collect only type of raw_data
    std::thread thread_id;
    geometry_msgs::Vector3Stamped message;
    geometry_msgs::Vector3Stamped temp_message;
    message.header.frame_id = "base_dvl";
    zeabus::service::get_data::GeometryVector3Stamped dvl_server( ptr_node_handle );
    dvl_server.register_data( &message );
    dvl_server.setup_ptr_mutex_data( ptr_mutex_data );
    dvl_server.setup_server_service( "/sensor/dvl");
    int temp_velocity[4] = { 0 , 0 , 0 , 0 }; // for collect data from function
    char ok_data;

    if( ptr_node_handle->ok() )
    {
        dvl_node.spin();
    }

    while( ptr_node_handle->ok() )
    {
        (void)dvl.read_data( &raw_data );
        type_line.clear() ; // make string are empty
        type_line.push_back( raw_data[1] );
        type_line.push_back( raw_data[2] );
        // get type of data
        if( type_line == "BS" )
        {
#ifdef _SUMMARY_
            zeabus::escape_code::clear_screen();
            std::cout   << "Raw data is " << raw_data << "\n";
#endif // _SUMMARY_
            zeabus::sensor::DVL::PD6_code_BS( &raw_data , &(temp_velocity[0]) 
                    , &(temp_velocity[1]) , &(temp_velocity[2]) , &ok_data );
            if( ok_data == 'A' ) // if data BS is ok
            {
                helper_status( true );
#ifdef _SUMMARY_
                std::cout   << "DVL GOOD DATA\n";
                std::cout   << "data is " << temp_velocity[0] << " " << temp_velocity[1]
                            << " " << temp_velocity[2] << "\n";
#endif // _SUMMARY_
                temp_message.vector.x = temp_velocity[0];
                temp_message.vector.y = temp_velocity[1];
                temp_message.vector.z = temp_velocity[2];
                temp_message.header.stamp = ros::Time::now();
                ptr_mutex_data->lock();
                message = temp_message;
                ptr_mutex_data->unlock();
				dvl_publisher.publish(message);
            }
            else
            {
                helper_status( false );
#ifdef _SUMMARY_
                std::cout << "DVL BAD DATA\n" ;
#endif
            }
        } // condition BS data
    } // while loop of ros operating system

    dvl.close_port();

    ros::shutdown();

    dvl_node.join();
    
    return 0;
} // function main

void helper_status( bool data )
{
    static bool status = false;
    if( ! status )
    {
        if( data )
        {
            ROS_INFO( "DVL STREAM DATA" );
            status = true;
        }
    }
    else
    {
        if( ! data )
        {
            ROS_FATAL( "DVL CAN'T STREAM DATA" );
            status = false;
        }
    }
}
