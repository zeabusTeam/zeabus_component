// FILE			: interfaces.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 25 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
// _SHOW_DATA_  : You will see all data to help you check calculate
// _SHOW_RPY_   : This will print RPY of target and current
// _CHECK_FIND_ERROR_RPY_ : This will show about data form find error RPY

// README
//  This file will use to calculate about error by 10 Hz

// REFERENCE

// MACRO SET
#define _SHOW_DATA_
#define _SHOW_RPY_
#define _CHECK_FIND_ERROR_RPY_

// MACRO CONDITION

#include    <array>

#include    <memory>

#include    <iostream>

#include    "read_data.cpp"

#include    <zeabus/time.hpp>

#include    <zeabus/count.hpp>

#include    <zeabus/radian.hpp>

#include    <geometry_msgs/Point.h>

#include    <zeabus/escape_code.hpp>

#include    <tf/LinearMath/Matrix3x3.h>

#include    <zeabus_utility/AUVState.h>

#include    <tf/LinearMath/Quaternion.h>

#include    <zeabus/ros_interfaces/time.hpp>

#include    <zeabus_utility/ControlCommand.h>

#include    <zeabus/service/control_command.hpp>

#include    <zeabus/ros_interfaces/single_thread.hpp>

#include    <zeabus/client/single_thread/get_auv_state.hpp>

#include    <zeabus/ros_interfaces/convert/geometry_msgs.hpp>

#include    <zeabus/client/single_thread/send_control_command.hpp>

int main( int argv , char** argc )
{
    // First part is about base code in ros system
    zeabus::ros_interfaces::SingleThread node_control_interfaces( argv 
            , argc , "control_interfaces" );

    std::shared_ptr< ros::NodeHandle > ptr_node_handle = 
            std::make_shared< ros::NodeHandle >("");

    std::shared_ptr< std::mutex > ptr_mutex_data = std::make_shared< std::mutex >();

    // Insert optional part param part
    const static unsigned int frequency = 10;
    static ros::Time time_stamp = ros::Time::now();
    zeabus_utility::ControlCommand command; // receive command 
    zeabus_utility::ControlCommand error; // send error

    // Second part setup varialbe have to use
    zeabus_utility::AUVState target_state; // collect target state
    zeabus_utility::AUVState current_state; // collect current state
    tf::Quaternion current_quaternion(0 , 0 , 0 , 1); // use to collect current quaternion
    tf::Quaternion target_quaternion(0 , 0 , 0 , 1); // use to collect target quaternion
    geometry_msgs::Point* ptr_target_position = &target_state.data.pose.pose.position; 
    geometry_msgs::Point* ptr_current_position = &current_state.data.pose.pose.position; 
    tf::Quaternion diff_quaternion(0 , 0 , 0 , 1); 
    // use to collect target_qua * current_qua.inv()
    std::array< double , 6 > buffer = {0, 0, 0, 0, 0, 0}; // will always target
#ifdef _SHOW_DATA_
    static double euler[3] = { 0 , 0 , 0 };
#endif
    bool force_target = true; // this mean we will use current state to target immediately 
    ros::Rate rate( frequency );
    // this buffer will use to compare with command

    // third part setup service
    zeabus::service::ControlCommand server_control_interfaces;
    server_control_interfaces.setup_ptr_node_handle( ptr_node_handle );
    server_control_interfaces.setup_ptr_mutex_data( ptr_mutex_data );
    server_control_interfaces.setup_ptr_data( &command );
    server_control_interfaces.setup_server_service( "/control/interfaces" );

    // Forth part setup client get current state
    zeabus::client::single_thread::GetAUVState client_control_state;
    client_control_state.setup_ptr_node_handle( ptr_node_handle );
    client_control_state.setup_ptr_data( &current_state );
    client_control_state.setup_client( "/fusion/auv_state" );

    // Fifth part setup client send error command
    zeabus::client::single_thread::SendControlCommand client_control_command;
    client_control_command.setup_ptr_node_handle( ptr_node_handle );
    client_control_command.setup_ptr_data( &error ); // error use ConttrolCommand
    client_control_command.setup_client( "/control/fuzzy" );  // set topic where error will send

    node_control_interfaces.spin();
    std::cout   << "Start loop\n";
    while( ptr_node_handle->ok() )
    {
        rate.sleep();
#ifdef _SHOW_DATA_
        zeabus::escape_code::clear_screen();
#endif

        (client_control_state).normal_call();

        // loop part : zero step tune current state
        zeabus::ros_interfaces::convert::quaternion_tf( 
                &current_state.data.pose.pose.orientation , &current_quaternion ); 
        if( force_target )
        {
            force_target = false;
            target_state.data = current_state.data;
            zeabus::ros_interfaces::convert::quaternion_tf( 
                    &target_state.data.pose.pose.orientation , &target_quaternion ); 
            tf::Matrix3x3( target_quaternion ).getRPY( buffer[3] , buffer[4] , buffer[5] );
            continue;
        } // one time delay

        // loop part : first compare about new command?
        ptr_mutex_data->lock();
        for(unsigned int run = 0 ; run < 6 ; run++ )
        {
            if( (command.mask)[ run ] )
            {
                buffer[ run ] = (command.target)[run];
                (command.mask)[run] = false; // We already known this command
            }
        }
        ptr_mutex_data->unlock();

        // loop part : second set a target
        target_quaternion.setRPY( buffer[3] , buffer[4] , buffer[5] );
        target_state.data.pose.pose.position.x = buffer[0];
        target_state.data.pose.pose.position.y = buffer[1];
        target_state.data.pose.pose.position.z = buffer[2];
        zeabus::ros_interfaces::convert::tf_quaternion( 
                &target_quaternion , &target_state.data.pose.pose.orientation );

        // loop part : third find different
        (error.target)[0] = ptr_target_position->x - ptr_current_position->x;
        (error.target)[1] = ptr_target_position->y - ptr_current_position->y;
        (error.target)[2] = ptr_target_position->z - ptr_current_position->z;
        diff_quaternion = target_quaternion * current_quaternion.inverse();
        tf::Matrix3x3( diff_quaternion ).getRPY( 
                (error.target)[3] , (error.target)[4] , (error.target)[5] );

#ifdef _CHECK_FIND_ERROR_RPY_
        std::cout   << "Before " << error.target[3] << " " << error.target[4] << " "
                    << error.target[5] << "\n";
#endif 
        zeabus::radian::bound( &( ( error.target )[3] ) );
        zeabus::radian::bound( &( ( error.target )[4] ) );
        zeabus::radian::bound( &( ( error.target )[5] ) );
#ifdef _CHECK_FIND_ERROR_RPY_
        std::cout   << "After " << error.target[3] << " " << error.target[4] << " "
                    << error.target[5] << "\n";
#endif

        // loop part : forth status of state decision
        //  state you can look by use binay format 3 bit
        //  0bpid   --> 0b is show you that is binary
        //          --> d that is dvl status
        //          --> i that is imu status
        //          --> p that is pressure status

        error.mask.assign( false );

        if( (current_state.status & 0b011 ) == 3 ) // dvl and imu and ok
        {
            (error.mask)[0] = true;
            (error.mask)[1] = true;
            (error.mask)[3] = true;
            (error.mask)[4] = true;
            (error.mask)[5] = true;
        }
        else if( (current_state.status & 0b010 ) == 2 ) // only imu ok
        {
            (error.mask)[3] = true;
            (error.mask)[4] = true;
            (error.mask)[5] = true;
        }
        else
        {
            ;
        }
        if( (current_state.status & 0b100 ) == 4 ) // only pressure ok
        {
            (error.mask)[2] = true;
        }
        
        // loop part : send error command
        client_control_command.normal_call();
#ifdef _SHOW_DATA_
        std::cout   << "\n--------------------------------------------------------------\n"
                    << "current position : " << ptr_current_position->x << " " 
                    << ptr_current_position->y << " " << ptr_current_position->z
                    << "\ntarget position  : " << ptr_target_position->x << " "
                    << ptr_target_position->y << " " << ptr_target_position->z
                    << "\nerror potision   : " << (error.target)[0] << " " << (error.target)[1]
                    <<  " " << (error.target)[2] << std::endl;
    #ifdef _SHOW_RPY_
        tf::Matrix3x3( current_quaternion ).getRPY( euler[0] , euler[1] , euler[2] );
        std::cout   << "current RPY: " << euler[0] << " " << euler[1] << " " << euler[2] << "\n";
        tf::Matrix3x3( target_quaternion ).getRPY( euler[0] , euler[1] , euler[2] );
        std::cout   << "target RPY : " << euler[0] << " " << euler[1] << " " << euler[2] << "\n"
                    << "diff RPY   : " << (error.target)[3] << " " << (error.target)[4]
                    << " " << (error.target)[5] << "\n";
    #else
        std::cout   << "current quaternion : " << current_quaternion.x() << " " 
                    << current_quaternion.y() << " " << current_quaternion.z() << " "
                    << current_quaternion.w() << "\n"
                    << "target quaternion : " << target_quaternion.x() << " " 
                    << target_quaternion.y() << " " << target_quaternion.z() << " "
                    << target_quaternion.w() << "\n"
                    << "diff quaternion : " << diff_quaternion.x() << " " 
                    << diff_quaternion.y() << " " << diff_quaternion.z() << " "
                    << diff_quaternion.w() << "\n";
    #endif
        std::cout   << "STATUS OF STATE " << read_bit_value( current_state.status )
                    << "\nMASK : " 
                    << read_bool( (error.mask)[0] ) << " " 
                    << read_bool( (error.mask)[1] ) << " " 
                    << read_bool( (error.mask)[2] ) << " " 
                    << read_bool( (error.mask)[3] ) << " " 
                    << read_bool( (error.mask)[4] ) << " " 
                    << read_bool( (error.mask)[5] )<< "\n";        
#endif 
    }
    ros::shutdown();
    node_control_interfaces.join();
    return 0; 
}

