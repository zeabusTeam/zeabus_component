// FILE         : imu_node.cpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 25
// MAINTAINER   : Supasan Komonlit

// MACRO DETAIL
//  _DECLARE_PROCESS_   : this macro will declare your process for debug about setting part
//  _PRINT_DATA_STREAM_ : this macro will declare process about data stream after you have 
//                          ability to read data from IMU
//  _DECLARE_UPDATED_    : this macro will declate you can't update data or not

//#define _DECLARE_PROCESS_ 

//#define _PRINT_DATA_STREAM_

#define _DECLARE_UPDATED_

#include    <ros/ros.h>

#include    <zeabus/sensor/IMU/connector.hpp>

#include    <zeabus/sensor/IMU/LORD_IMU_COMMUNICATION.hpp>

#include    <zeabus/convert/vector/one_byte.hpp>

#include    <zeabus/ros_interfaces/single_thread.hpp>
#include    <zeabus/service/get_data/sensor_imu.hpp>

#include    <iostream>
#include    <vector>

#include    <sensor_msgs/Imu.h>

#include    <zeabus/escape_code.hpp>

// FINSISH SETUP IMU LINE 180

namespace Asio = boost::asio;
namespace IMUProtocal = zeabus::sensor::IMU::LORD_MICROSTRAIN;

int main( int argv , char** argc )
{
    zeabus::sensor::IMU::Connector imu("/dev/microstrain/3dm_gx5_45_0000__6251.65903" , 100 );

    zeabus::ros_interfaces::SingleThread imu_node( argv , argc , "imu_node");

    std::shared_ptr< ros::NodeHandle > ptr_node_handle = 
            std::make_shared< ros::NodeHandle >("");

    std::shared_ptr< std::mutex > ptr_mutex_data = std::make_shared< std::mutex >();

#ifdef _DECLARE_PROCESS_
    std::cout   << "Finish declare basic object of IMU node\n";
#endif // _DECLARE_PROCESS_

    bool status_file = true ; // use collect response of function
    bool skip_process = false; // use to don't do that process and don't alert
    unsigned int round = 0;
    unsigned int limit_round = 6; // if you want to try n round set limit_round = n + 1

    status_file = imu.open_port();
    if(  ! status_file )
    {
        std::cout << "Failure to open port imu\n";
        skip_process = true;
    }
#ifdef _DECLARE_PROCESS_
    else{
        std::cout << "Finish open_port process\n";
    }
#endif // _DECLARE_PROCESS_

	(void)imu.set_option_port( Asio::serial_port_base::flow_control( 
							Asio::serial_port_base::flow_control::none ) );
	(void)imu.set_option_port( Asio::serial_port_base::parity( 
							Asio::serial_port_base::parity::none ) );
	(void)imu.set_option_port( Asio::serial_port_base::stop_bits( 
							Asio::serial_port_base::stop_bits::one ) );
	(void)imu.set_option_port( Asio::serial_port_base::character_size( (unsigned char) 8 ) );

#ifdef _DECLARE_PROCESS_
    std::cout << "Finish setup port of imu\n";
#endif // _DECLARE_PROCESS_

    round = 0; // set init value counter is 0 for start process
    while( ( ! skip_process ) && ptr_node_handle->ok() )
    {
        round++;
        status_file = imu.set_idle(); // try to set imu to idle state
        if( ! status_file )
        {
            std::cout   << "round " << round << " : Failure command set idle\n";
        }
        else
        {
            std::cout   << "round " << round << " : Success command set idle\n\n";
            break; // jump success this process
        }
        if( round == (limit_round * 20) )
        {
            skip_process = true;
        }
    }

    round = 0; // set init value counter is 0 for start process
    while( ( ! skip_process ) && ptr_node_handle->ok() )
    {
        round++;
        status_file = imu.ping();
        if( ! status_file )
        {
            std::cout   << "round " << round << " : Failure command ping\n";
        }
        else
        {
            std::cout   << "round " << round << " : Success command ping\n\n";
            break; // jump sunccess this process
        }
        if( round == limit_round )
        {
            skip_process = true;
        }
    }

    imu.set_IMU_rate( 50 ); // send in mode Rate Decimation = IMU Base Rate / Desired Data Rate

    round = 0;
    while( ( ! skip_process ) && ptr_node_handle->ok() )
    {
        round++;
        status_file = imu.set_IMU_message_format( 
                IMUProtocal::DATA::IMU_DATA_SET::SCALED_ACCELEROMETER_VECTOR 
                , IMUProtocal::DATA::IMU_DATA_SET::SCALED_GYRO_VECTOR
                , IMUProtocal::DATA::IMU_DATA_SET::CF_QUATERNION );
        if( ! status_file )
        {
            std::cout   << "round " << round << " : Failure command set IMU message format\n";
        }
        else
        {
            std::cout   << "round " << round << " : Success command set IMU message format\n";
            break;
        }
        if( round == limit_round )
        {
            skip_process = true;
        }
    }
    // we not save because we have new set up always want to use

    round = 0;
    while( ( ! skip_process ) && ptr_node_handle->ok() )
    {
        round++;
        status_file = imu.enable_IMU_data_stream();
        if( ! status_file )
        {
            std::cout   << "round " << round 
                        << " : Failure command set enable IMU data stream\n";
        }
        else
        {   
            std::cout   << "round " << round 
                        << " : Success command set enable IMU data stream\n";
            break;
        }
        if( round == limit_round )
        {
            skip_process = true;
        }
    }

    round = 0;
    while( ( ! skip_process ) && ptr_node_handle->ok() )
    {
        round++;
        status_file = imu.resume();
        if( ! status_file )
        {
            std::cout   << "round " << round 
                        << " : Failure command resume data stream\n";
        }
        else
        {
            std::cout   << "round " << round 
                        << " : Success command resume data stream\n";
            break;
        }
        if( round == limit_round )
        {
            skip_process = true;
        }
    }

#ifdef _DECLARE_PROCESS_
    std::cout   <<  "Now setup object for ROS Mode\n";
#endif // _DECLARE_PROCESS_

    sensor_msgs::Imu message;
    sensor_msgs::Imu temporary_message;
    message.header.frame_id = "imu";

    zeabus::service::get_data::SensorImu imu_server( ptr_node_handle );
    imu_server.register_data( &message );
    imu_server.setup_ptr_mutex_data( ptr_mutex_data );
    bool result = imu_server.setup_server_service( "/sensor/imu" );
    if( ! result )
    {
        std::cout   << "Failure setup service\n";
        skip_process = true;
    }

    if( ! skip_process ){
        result = imu_node.spin();
    }

    if( ! result )
    {
        std::cout   << "Unsucess spin\n";
        skip_process = true;
    }

#ifdef _DECLARE_PROCESS_
    std::cout   << "Now start streaming data\n";
#endif // _DECLARE_PROCESS_

    unsigned int limit_number;    
    while( ( ptr_node_handle->ok() && ( ! skip_process ) ) )
    {
#ifdef _PRINT_DATA_STREAM_
        std::cout   << "Read data form IMU\n";
#endif // _PRINT_DATA_STREAM_
        status_file = imu.read_stream();
        if( status_file )
        {
#ifdef _PRINT_DATA_STREAM_
            imu.print_data( "IMU message " );
#endif // _PRINT_DATA_STREAM_
            if( imu.access_data(2) != 0x80 ) // Desciptor set byte of data stream is 0x80
            {
                std::cout << "This not packet for data stream skip out\n";
                continue;
            }
            // start at position 5 indent 0 1 2 3  
            // because 0 - 4 is header and description of data packet
            // pattern of packet for 0 1 2 3 4 are u e DESC_Packet Payload_length Field_length
            limit_number = imu.size_member() - 2 ;
            for( unsigned int run = 5 ; ( run < limit_number ) && ( ! skip_process ) ; )
            {
                switch (imu.access_data( run ) )
                {
                case IMUProtocal::DATA::IMU_DATA_SET::SCALED_ACCELEROMETER_VECTOR :
                    zeabus::convert::vector::one_byte::vector3( &(imu.data) 
                            , &(temporary_message.linear_acceleration) , run + 1);
                    run += 14 ; // skip to point start data < 1 byte >
                                // skip to point legth data 3 floats < 12 bytes >
                                // skip for next field length < 1 byte>
                                // this will make run will point to next descriptor
                    break;
                case IMUProtocal::DATA::IMU_DATA_SET::SCALED_GYRO_VECTOR :
                    zeabus::convert::vector::one_byte::vector3( &(imu.data) 
                            , &(temporary_message.angular_velocity) , run + 1);
                    run += 14 ; // skip to point start data < 1 byte >
                                // skip to point legth data 3 floats < 12 bytes >
                                // skip for next field length < 1 byte>
                                // this will make run will point to next descriptor
                    break;
                case IMUProtocal::DATA::IMU_DATA_SET::CF_QUATERNION :
                    zeabus::convert::vector::one_byte::quaternion( &(imu.data) 
                            , &(temporary_message.orientation) , run + 1);
                    run += 18 ; // skip to point start data < 1 byte >
                                // skip to point legth data 4 floats < 12 bytes >
                                // skip for next field length < 1 byte>
                                // this will make run will point to next descriptor
                    break;
                default :
                    std::cout   << "Switch case error for convert bits data to ros message\n";
                    skip_process = true;
                    break;
                }
                temporary_message.header.stamp = ros::Time::now();
            } // loop for of get data
            if( ptr_mutex_data->try_lock() )
            {
                message = temporary_message;
                ptr_mutex_data->unlock();
#ifdef _DECLARE_UPDATED_
                std::cout   << zeabus::escape_code::bold_yellow
                            << "Update IMU data\n" << zeabus::escape_code::normal_white;
#endif // _DECLARE_UPDATED_
            }
#ifdef _DECLARE_UPDATED_
            else
            {
                std::cout   << zeabus::escape_code::bold_red  << "Did't update IMU data\n" 
                            << zeabus::escape_code::normal_white;
            }
#endif //  _DECLARE_UPDATED_ 
        } // condition have packet of data stream
#ifdef _DECLARE_UPDATED_
        else
        {
            std::cout   << zeabus::escape_code::bold_red << "<--- IMU ---> BAD DATA\n\n"
                        << zeabus::escape_code::normal_white;
        }
#endif // _DECLARE_UPDATED_
    } // loop while for doing in ros system

    round = 0; // set init value counter is 0 for start process
    while( imu.port_is_open() ) //
    {
        round++;
        status_file = imu.set_idle(); // try to set imu to idle state
        if( ! status_file )
        {
            std::cout   << "round " << round << " : Failure command set idle\n";
        }
        else
        {
            std::cout   << "round " << round << " : Success command set idle\n\n";
            break; // jump success this process
        }
    }

    std::cout   << "Now close port of imu\n";
    imu.close_port();

    // We want to ensure other thread have been close defend core dump
    std::cout   << "Wait join from thread\n";
    imu_node.join();

}
