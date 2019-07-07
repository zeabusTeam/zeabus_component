// FILE			: imu_capture_gyro.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, June 28 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <iostream>

#include    <ros/ros.h>

#include    <zeabus/sensor/IMU/interface.hpp>

#include    <zeabus/ros_interfaces/single_thread.hpp>

#include    <zeabus/sensor/IMU/LORD_IMU_COMMUNICATION.hpp>

namespace Asio = boost::asio;
namespace IMUProtocal = zeabus::sensor::IMU::LORD_MICROSTRAIN;

int main( int argv , char** argc )
{
    zeabus::ros_interfaces::SingleThread imu_node( argv , argc , "imu_capture_gyro_bias" );

    std::string full_path_port = "/dev/microstrain/3dm_gx5_45_0000__6251.65903";

    zeabus::sensor::IMU::Interface imu( full_path_port , 100 );

    std::shared_ptr< ros::NodeHandle > ptr_node_handle = 
            std::make_shared< ros::NodeHandle >("");

    unsigned int round = 0;
    unsigned int limit_round = 20; // if you want to try n round set limit_round = n + 1

    if(  ! imu.open_port() )
    {
        ROS_FATAL_NAMED( "SENSOR_IMU" ,  "Failure to open port %s" , full_path_port.c_str() );
        ros::shutdown();
    }

    if( ptr_node_handle->ok() )
    {
	    (void)imu.set_option_port( Asio::serial_port_base::flow_control( 
		        Asio::serial_port_base::flow_control::none ) );
	    (void)imu.set_option_port( Asio::serial_port_base::parity( 
				Asio::serial_port_base::parity::none ) );
	    (void)imu.set_option_port( Asio::serial_port_base::stop_bits( 
				Asio::serial_port_base::stop_bits::one ) );
	    (void)imu.set_option_port( Asio::serial_port_base::character_size( (unsigned char) 8 ) );
    }
/*
    round = 0; // set init value counter is 0 for start process
    while( ptr_node_handle->ok() )
    {
        round++;
        if( ! imu.set_idle() ) // try to set imu to idle state
        {
            std::cout   << "round " << round << " : Failure command set idle\n";
        }
        else
        {
            std::cout   << "round " << round << " : Success command set idle\n\n";
            break; // jump success this process
        }
        if( round == (limit_round) )
        {
            ros::shutdown();
        }
    }
*/
    bool result = imu.auto_set_gyro_bias( true );
    if( result )
    {
        std::cout   << "Main node finish\n";
    }
    else
    {
        std::cout   << "Main node finish and failute\n";
    }
    return 0;
}
