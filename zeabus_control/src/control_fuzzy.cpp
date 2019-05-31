// FILE			: control_fuzzy.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 27 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
// _SUMMARY_    : If you macro this, code will show you about input and output for connection
//              between interface and command thruster of control

// README

// REFERENCE

// MACRO SET
#define _SUMMARY_

// MACRO CONDITION

#include    <array>

#include    <mutex>

#include    <memory>

#include    <iostream>

#include    <ros/ros.h>

#include    <zeabus/count.hpp>

#include    <zeabus/escape_code.hpp>

#include    <tf/LinearMath/Matrix3x3.h>

#include    <zeabus/fuzzy/control_error.hpp>

#include    <zeabus_utility/ControlCommand.h>

#include    <zeabus/service/control_command.hpp>

#include    <zeabus/ros_interfaces/single_thread.hpp>

#include    <zeabus/client/single_thread/get_auv_state.hpp>

#include    <zeabus/ros_interfaces/convert/geometry_msgs.hpp>

#include    <zeabus/client/single_thread/send_control_command.hpp>

int main( int argv , char** argc )
{

    // First part is about base code in ros system
    zeabus::ros_interfaces::SingleThread node_control_fuzzy( argv , argc , "control_fuzzy" );

    std::shared_ptr< ros::NodeHandle > ptr_node_handle =
            std::make_shared< ros::NodeHandle >("");

    std::shared_ptr< std::mutex > ptr_mutex_data = std::make_shared< std::mutex >();

    // Insert optional part param part
    const static unsigned int frequency = 10;
    static ros::Time time_stamp = ros::Time::now();
    zeabus_utility::ControlCommand error; // received error
    zeabus_utility::ControlCommand force; // send_force
    zeabus_utility::ControlCommand temp;
    zeabus_utility::AUVState current_state;
    ros::Rate rate( frequency );

    zeabus::client::single_thread::GetAUVState client_control_state;
    client_control_state.setup_ptr_node_handle( ptr_node_handle );
    client_control_state.setup_ptr_data( &current_state );
    client_control_state.setup_client( "/fusion/auv_state" );

    // Part about fuzzy logic
    std::array< std::array< signed char , 7 > , 7 > fuzzy_rule = {
            1 , 1 , 2 , 3 , 3 , 3 , 3 
            , -1 , 0 , 1 , 2 , 2 , 3 , 3
            , -2 , 1 , 0 , 1 , 2 , 3 , 3
            , -2 , -2 , -1 , 0 , 1 , 2 , 2
            , -3 , -3 , -2 , -1 , 0 , 1 , 2 
            , -3 , -3 , -2 , -2 , -1 , 0 , 1
            , -3 , -3 , -3 , -3 , 2 , 1 , 1
    };

    zeabus::fuzzy::ControlError<5> fuzzy_logic[6];
    
    // Pattern is x y z roll pitch yaw
    double offset_value[6] = { 0 , 0 , -1 , 0 , 0 , 0 };
    // The next 1 type have 3 value
    double relative_value[18] = { 0.05 , 0.1 , 0.2 
            , 0.05 , 0.1 , 0.2
            , 0.1 , 0.15 , 0.3 
            , 0.01 , 0.05 , 0.1
            , 0.01 , 0.05 , 0.1
            , 0.05 , 0.1 , 0.15 };

    double error_range[18] = { 0.05 , 1.5 , 5 
            , 0.05 , 1.5 , 5 
            , 0.1 , 2 , 3 
            , 0.1 , 1 , 2 
            , 0.1 , 1 , 2 
            , 0.1 , 1 , 2 };

    double diff_range[18] = { 0.05 , 0.1 , 0.2 
            , 0.05 , 0.1 , 0.2 
            , 0.1 , 0.2 , 0.4 
            , 0.01 , 0.15 , 0.3
            , 0.01 , 0.15 , 0.3
            , 0.05 , 0.2 , 0.5 };

    double force_range[18]; // not use now

    for( unsigned int run = 0 ; run < 6 ; run++ )
    {
        fuzzy_logic[ run ].set_rule_condition( &fuzzy_rule );
        fuzzy_logic[ run ].set_offset_force( offset_value[ run ] );
        fuzzy_logic[ run ].set_relative_value( relative_value[ run * 3 + 0 ] 
                , relative_value[ run * 3 + 1 ] , relative_value[ run * 3 + 2 ] );
        fuzzy_logic[ run ].set_error_range( error_range[ run * 3 + 0 ] 
                , error_range[ run * 3 + 1 ] , error_range[ run * 3 + 2 ] );
        fuzzy_logic[ run ].set_diff_range( diff_range[ run * 3 + 0 ] 
                , diff_range[ run * 3 + 1 ] , diff_range[ run * 3 + 2 ] );
        fuzzy_logic[ run ].clear_system();
    }

    // part of client
    zeabus::client::single_thread::SendControlCommand client_control_fuzzy;
    client_control_fuzzy.setup_ptr_node_handle( ptr_node_handle );
    client_control_fuzzy.setup_ptr_data( &error );
    client_control_fuzzy.setup_client( "/control/thruster");

    // part of service
    zeabus::service::ControlCommand server_control_fuzzy;
    server_control_fuzzy.setup_ptr_node_handle( ptr_node_handle );
    server_control_fuzzy.setup_ptr_mutex_data( ptr_mutex_data );
    server_control_fuzzy.setup_server_service( "/control/fuzzy" );

    node_control_fuzzy.spin();
    tf::Quaternion temp_quaternion;
    tf::Quaternion state_quaternion;
    while( ptr_node_handle->ok() )
    {
        rate.sleep();
#ifdef _SUMMARY_
        zeabus::escape_code::clear_screen();
#endif // _SUMMARY_
        client_control_state.normal_call();
        ptr_mutex_data->lock();
        temp = error;
        ptr_mutex_data->unlock();
        for( unsigned int run = 0 ; run <6 ; run++ )
        {
            if( (force.mask)[run] )
            {
                (force.target)[ run ] = fuzzy_logic[ run ].push( (temp.target)[run] );
            }
            else
            {
                fuzzy_logic[ run ].clear_system();
                (force.target)[ run ] = 0;
            }
        }
        
        temp_quaternion = tf::Quaternion( (force.target)[0] , (force.target)[1]
                , (force.target)[2] , 0 );
        zeabus::ros_interfaces::convert::tf_quaternion( &state_quaternion 
                , &current_state.data.pose.pose.orientation );

        temp_quaternion = state_quaternion * temp_quaternion * state_quaternion.inverse();

        (force.target)[ 0 ] = temp_quaternion.x();
        (force.target)[ 1 ] = temp_quaternion.y();
        (force.target)[ 2 ] = temp_quaternion.z();

#ifdef _SUMMARY_
        std::cout   << "Input\tMask\tOutput\n";
        for( unsigned int run = 0 ; run < 6 ; run++ )
        {
            std::cout   << temp.target[run] << "\t" << temp.mask[run] << "\t" 
                        << force.target[run] << "\n";
        } // loop for for print summary case macro
#endif // _SUMMARY_
        client_control_fuzzy.normal_call();
    }
    ros::shutdown();
    node_control_fuzzy.join();
    return 0;
}
