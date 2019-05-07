// FILE         : maestro_pololu_node.hpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, MAY 2
// MAINTAINER   : Supasan Komonlit

#include    <iostream>

#include    <mutex>

#include    <memory>

#include    <vector>

#include    <zeabus/escape_code.hpp>

#include    <zeabus/hardware/MAESTRO/connector.hpp>

#include    <ros/ros.h>

#include    <zeabus/service/pololu.hpp>

#include    <zeabus/ros_interfaces/single_thread.hpp>

// MACRO_DEATAIL
// _PRINT_PROCESS_      : will print all process of in function main of this node
// _PRINT_DATA_WRITING_ : will print all PWM have been write to serial
// _PRINT_SUCCESS_WRITE_: will print if you success writing data to serial port 
// _PRINT_BEFORE_SET_   : will print array of target PWM before set value

#define _PRINT_DATA_WRITING_
#define _PRINT_SUCCESS_WRITE_
#define _PRINT_BEFORE_SET_

namespace Asio = boost::asio;

void reset_value( std::vector< unsigned short int >* buffer , unsigned short int value 
        , unsigned int size );

int main( int argv , char** argc )
{

    // Below line will init_node and object spin in new thread
    zeabus::ros_interfaces::SingleThread maestro_node( argv , argc , "maestro_node" );
 
    std::shared_ptr< ros::NodeHandle > ptr_node_handle = std::make_shared< ros::NodeHandle>("");

    std::shared_ptr< std::mutex > ptr_mutex_data = std::make_shared< std::mutex >();

    static const std::string port_name = "/dev/pololu/maestro/00185168";

    zeabus::hardware::MAESTRO::POLOLU::Connector maestro_port( port_name );

    zeabus::service::Pololu maestro_server( ptr_node_handle );

    bool status_node;

    unsigned int thruster_size = 8 ;

    status_node = maestro_port.open_port();
    if( ! status_node )
    {
        std::cout   << zeabus::escape_code::bold_red << "Can't open port pololu name "
                    << port_name << "\n" << zeabus::escape_code::normal_white;
    }
#ifdef _PRINT_PROCESS_
    else
    {
        std::cout   << "Success open port pololu\n";
    }
    #endif // _PRINT_PROCESS_
    if( status_node )
    {
        (void)maestro_port.set_option_port( Asio::serial_port_base::flow_control(
                Asio::serial_port_base::flow_control::none ) );
        (void)maestro_port.set_option_port( Asio::serial_port_base::parity(
                Asio::serial_port_base::parity::none ) );
        (void)maestro_port.set_option_port( Asio::serial_port_base::stop_bits(
                Asio::serial_port_base::stop_bits::one ) );
        (void)maestro_port.set_option_port( Asio::serial_port_base::character_size( (unsigned char) 8) );
        (void)maestro_port.set_option_port( Asio::serial_port_base::baud_rate( 115200 ) );
#ifdef _PRINT_PROCESS_
        std::cout   << "Finish setup port pololu\n";
#endif // _PRINT_PROCESS_
    } // condition for set up port

    std::vector< unsigned short int> target_pwm;
    std::vector< unsigned short int> temp_target_pwm;
    if( status_node )
    {
        reset_value( &target_pwm , 1500 , thruster_size );
        reset_value( &temp_target_pwm , 1500 , thruster_size );
        maestro_port.set_multiple_targets( &target_pwm );
    }

    ros::Rate rate( 10 );
    ros::Time time_updated;

    double time_different;    
    double timeout = 1;

    maestro_server.setup_ptr_mutex_data( ptr_mutex_data );
    maestro_server.setup_ptr_data( &temp_target_pwm , thruster_size , &time_updated );

    bool node_status = maestro_server.setup_server_service( "/hardware/pwm");
    if( !node_status )
    {
        status_node = false;
    }
#ifdef _PRINT_PROCESS_
    else
    {
        std::cout   << "Success setup server service\n";
    }
#endif

    node_status = maestro_node.spin();
    if( ! node_status )
    {
        status_node = false;
    }
#ifdef _PRINT_PROCESS_
    else
    {
        std::cout   << "Success open node spin\n";
    }
#endif // _PRINT_PROCESS_
   
#ifdef _PRINT_SUCCESS_WRITE_
    bool write_success;
#endif // _PRINT_SUCCESS_WRITE_
 
    while( ptr_node_handle->ok() && node_status )
    {
        rate.sleep();
        ptr_mutex_data->lock();
        // http://docs.ros.org/api/rostime/html/classros_1_1TimeBase.html reference toSec()
        //  ros::Time is subclass of ros::TimeBase
        time_different = (ros::Time::now() - time_updated).toSec();
        if( time_different < timeout )
        {
            for( unsigned int run = 0 ; run < thruster_size ; run++ )
            {
                target_pwm[run] = temp_target_pwm[run];
            } // function copy value
        }
        ptr_mutex_data->unlock(); 
        if( time_different > timeout )
        {
            reset_value( &target_pwm , 1500 , thruster_size );
            std::cout   << zeabus::escape_code::normal_yellow << "Time out of PWM command "
                        << timeout << " now go home\n" << zeabus::escape_code::normal_white;
        }
#ifdef _PRINT_DATA_WRITING_
        std::cout   << "------------------------------------------------------------\n";
        for( unsigned int run = 0 ; run < thruster_size ; run++ )
        {
            std::cout   << "\t" << target_pwm[ run ];
            if( run == 3 )
            {
                std::cout << "\n";
            }
        }
        std::cout   << "\n------------------------------------------------------------\n";
#endif // _PRINT_DATA_WRITING_
#ifdef _PRINT_BEFORE_SET_
        std::cout   << "---------------------TARGET PWM--------------------------------\n";
        for( unsigned int run = 0 ; run < thruster_size ; run++ )
        {
            std::cout   << "  " << target_pwm[run]; 
        }
        std::cout   << "\n---------------------------------------------------------------\n";
#endif // _PRINT_BEFORE_SET_ 
#ifdef _PRINT_SUCCESS_WRITE_
        write_success = maestro_port.set_multiple_targets( &target_pwm );
        if( write_success )
        {
            std::cout   << "Success writing data\n";
        }
        else
        {
            std::cout   << zeabus::escape_code::bold_red << "Failure writing data\n"
                        << zeabus::escape_code::normal_white;
        }
#else
        (void)maestro_port.set_multiple_targets( &target_pwm );
#endif // _PRINT_SUCCESS_WRITE_
    } // while ros loop

    std::cout   << "Now close port of maestro\n";
    maestro_port.close_port();

    if( status_node )
    {
        std::cout   << "Wait join from thread\n";
        maestro_node.join();
    }

    return 0;
} // function main

void reset_value( std::vector< unsigned short int >* buffer , unsigned short int value 
        , unsigned int size )
{
    if( buffer->size() < size )
    {
        buffer->resize( size ); // must ensure you have size to do this
    }
    for( unsigned int run = 0 ; run < size ; run++ )
    {
        (*buffer)[ run ] = value;
    }
}
