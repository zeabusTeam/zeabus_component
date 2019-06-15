// FILE         : imu_node.cpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 25
// MAINTAINER   : Supasan Komonlit

// MACRO DETAIL
//  _DECLARE_PROCESS_   : this macro will declare your process for debug about setting part
//  _PRINT_DATA_STREAM_ : this macro will declare process about data stream after you have 
//                          ability to read data from IMU
//  _DECLARE_UPDATED_   : this macro will declate you can't update data or not
//  _SUMMARY_           : this macro will include _DECLARE_UPDATED_ and show last result
//                          now available only euler
//  _IMU_ENU_SYSTEM_    : if you macro this. This programe will not convert data from NED to ENU

// MACRO SET
//#define _DECLARE_PROCESS_ 
//#define _PRINT_DATA_STREAM_
//#define _DECLARE_UPDATED_
//#define _SUMMARY_
#define _IMU_ENU_SYSTEM_

// MACRO CONDITION
#ifdef _SUMMARY_
    #define _DECLARE_UPDATED_
#endif

#ifdef _IMU_ENU_SYSTEM_
    #define _SUMMARY_
#endif

#include    <vector>

#include    <iostream>

#include    <ros/ros.h>

#include    <ros/console.h>

#include    <sensor_msgs/Imu.h>

#include    <zeabus/escape_code.hpp>

#include    <tf/LinearMath/Matrix3x3.h>

#include    <tf/LinearMath/Quaternion.h>

#include    <zeabus/sensor/IMU/connector.hpp>

#include    <zeabus/convert/vector/one_byte.hpp>

#include    <zeabus/service/get_data/sensor_imu.hpp>

#include    <zeabus/ros_interfaces/single_thread.hpp>

#include    <zeabus/sensor/IMU/LORD_IMU_COMMUNICATION.hpp>

#include    <zeabus/ros_interfaces/convert/geometry_msgs.hpp>

namespace Asio = boost::asio;
namespace IMUProtocal = zeabus::sensor::IMU::LORD_MICROSTRAIN;

void helper_status( bool data );

int main( int argv , char** argc )
{

    zeabus::ros_interfaces::SingleThread imu_node( argv , argc , "imu" );

    ros::NodeHandle param_handle("~");

    std::string full_path_port;
    param_handle.param< std::string>( "full_path_port"
        , full_path_port
        , "/dev/microstrain/3dm_gx5_45_0000__6251.65903");

    zeabus::sensor::IMU::Connector imu( full_path_port , 100 );

#ifdef _IMU_ENU_SYSTEM_
    std::string node_name = "imu_node_ned";
    const static tf::Quaternion convert_conventions( 0.7071 , 0.7071 , 0 , 0 );
#else
    std::string node_name = "imu_node";
#endif

    std::shared_ptr< ros::NodeHandle > ptr_node_handle = 
            std::make_shared< ros::NodeHandle >("");

    std::shared_ptr< std::mutex > ptr_mutex_data = std::make_shared< std::mutex >();

	ros::Publisher imu_publisher = ptr_node_handle->advertise<sensor_msgs::Imu>("/sensor/imu", 1);

#ifdef _DECLARE_PROCESS_
    std::cout   << "Finish declare basic object of IMU node\n";
#endif // _DECLARE_PROCESS_

    bool status_file = true ; // use collect response of function
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

#ifdef _DECLARE_PROCESS_
    std::cout << "Finish setup port of imu\n";
#endif // _DECLARE_PROCESS_

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

    if( ptr_node_handle->ok() )
    {
        (void)imu.ping();

        imu.set_IMU_rate( 50 ); 
        (void)imu.set_IMU_message_format( 
                IMUProtocal::DATA::IMU_DATA_SET::SCALED_ACCELEROMETER_VECTOR 
                , IMUProtocal::DATA::IMU_DATA_SET::SCALED_GYRO_VECTOR
                , IMUProtocal::DATA::IMU_DATA_SET::CF_QUATERNION );

        (void)imu.enable_IMU_data_stream();

        (void)imu.resume();
    }

#ifdef _DECLARE_PROCESS_
    std::cout   <<  "Now setup object for ROS Mode\n";
#endif // _DECLARE_PROCESS_

    sensor_msgs::Imu message;
    sensor_msgs::Imu temporary_message;
    message.header.frame_id = "base_imu";

    zeabus::service::get_data::SensorImu imu_server( ptr_node_handle );
    imu_server.register_data( &message );
    imu_server.setup_ptr_mutex_data( ptr_mutex_data );
    (void)imu_server.setup_server_service( "/sensor/imu" );

#ifdef _DECLARE_PROCESS_
    std::cout   << "Now start streaming data\n";
#endif // _DECLARE_PROCESS_

#ifdef _SUMMARY_
    tf::Quaternion temp_quaternion;
    double temp_RPY[3];
#endif

    imu_node.spin();
    unsigned int limit_number = 10;    
    while( ptr_node_handle->ok() )
    {
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
            for( unsigned int run = 5 ; ( run < limit_number ) && ptr_node_handle->ok() ; )
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
                    ros::shutdown();
                    break;
                }
                temporary_message.header.stamp = ros::Time::now();
            } // loop for of get data

            // Below macro condtion if use set IMU is NED conventions we will convert before out
#ifdef _SUMMARY_
            zeabus::escape_code::clear_screen();
#endif // _SUMMARY_ 
#ifdef _IMU_ENU_SYSTEM_
            zeabus::ros_interfaces::convert::quaternion_tf(
                    &temporary_message.orientation , &temp_quaternion );
            tf::Matrix3x3( temp_quaternion ).getRPY( temp_RPY[0], temp_RPY[1], temp_RPY[2] );
            std::cout   << "NED system : " << temp_RPY[0] << " " << temp_RPY[1] 
                        << " " << temp_RPY[2] << "\n";
            temp_quaternion = convert_conventions*temp_quaternion*convert_conventions.inverse();
            std::cout   << "Use quaternion convert : " << convert_conventions.x() << " "
                        << convert_conventions.y() << " " << convert_conventions.z() 
                        << " " << convert_conventions.w() << "\n";
            zeabus::ros_interfaces::convert::tf_quaternion(
                    &temp_quaternion , &temporary_message.orientation );
#endif

            helper_status( true );
            ptr_mutex_data->lock();
            message = temporary_message;
            ptr_mutex_data->unlock();
			imu_publisher.publish(message);

#ifdef _SUMMARY_
            zeabus::ros_interfaces::convert::quaternion_tf( 
                    &temporary_message.orientation 
                    , &temp_quaternion );
            tf::Matrix3x3( temp_quaternion ).getRPY( temp_RPY[0], temp_RPY[1], temp_RPY[2] );
            std::cout   << "Data in Euler is " << temp_RPY[0] << " " << temp_RPY[1] 
                        << " " << temp_RPY[2] << "\n";
#endif

#ifdef _DECLARE_UPDATED_
            std::cout   << zeabus::escape_code::bold_yellow
                        << "Update IMU data\n" << zeabus::escape_code::normal_white;
#endif // _DECLARE_UPDATED_

        } // condition have packet of data stream
#ifdef _DECLARE_UPDATED_
        else
        {
            helper_status( false );
        }
#endif // _DECLARE_UPDATED_
    } // loop while for doing in ros system

    // Below step we want to ensure in case we not use imu. Imu will stop process data stream
    round = 0; // set init value counter is 0 for start process
    while( imu.port_is_open() ) 
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

    ros::shutdown();

    imu.close_port();
    std::cout   << "Now close port of imu\n";

    // We want to ensure other thread have been close defend core dump
    imu_node.join();
    std::cout   << "finish join from thread\n";

    return 0;
}

void helper_status( bool data )
{
    static bool status = false;
    if( status ) // If past event is good data
    {
        if( ! data ) // But now are bad data
        {
            ROS_DEBUG_NAMED("SENSOR_IMU" , "IMU can't streaming data" );
            status = false;
        }
    }
    else
    {
        if( data )
        {
            ROS_DEBUG_NAMED("SENSOR_IMU" , "IMU now are streaming data" );
            status = true;
        }
    }
}
