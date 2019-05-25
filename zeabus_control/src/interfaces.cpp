// FILE			: interfaces.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 25 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This file will use to calculate about error by 10 Hz

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <memory>

#include    <iostream>

#include    <zeabus/time.hpp>

#include    <zeabus/count.hpp>

#include    <tf/LinearMath/Matrix3x3.h>

#include    <tf/LinearMath/Quaternion.h>

#include    <zeabus_utility/AUVState.h>

#include    <zeabus/client/single_thread/get_auv_state.hpp>

#include    <zeabus_utility/ControlCommand.h>

#include    <zeabus/service/control_command.hpp>

#include    <zeabus/ros_interfaces/single_thread.hpp>

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
    tf::Quaternion current_quaternion(0 , 0 , 0); // use to collect current quaternion
    tf::Quaternion target_quaternion(0 , 0 , 0); // use to collect target quaternion
    tf::Quaternion diff_quaternion(0 , 0 , 0);  // use to collect target_qua * current_qua.inv()

    // third part setup service
    zeabus::service::ControlCommand server_control_interfaces;
    server_control_interfaces.setup_ptr_node_handle( ptr_node_handle );
    server_control_interfaces.setup_ptr_mutex_data( ptr_mutex_data );
    server_control_interfaces.setup_ptr_data( &command );

    // Forth part setup client get current state
    zeabus::client::single_thread::GetAUVState client_control_interfaces;
    client_control_interfaces.setup_ptr_node_handle( ptr_node_handle );
    client_control_interfaces.setup_ptr_data( &current_state );

    // Fifth part setup client send error command
    
}
