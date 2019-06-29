// FILE			: main_control.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, June 22 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
// _SUMMARY_    : If you macro this, code will show you about input and output for connection
//              between interface and command thruster of control
// _SHOW_ROTATION_
//              : Will show you about rotation from output in world frame to robot frame
// --> WARNING MACRO OF FUZZY LIB
// _SHOW_DATA_  : This is macro will show you see about process of fuzzy logic and your data
// _SHOW_RULE_TABLE_
//              : This macro will alway show data of multiple index data

// README
//  This is the lastest fuzzy control on ZEABUS Team. This version 2

// REFERENCE

// MACRO SET
#define _SUMMARY_
//#define _SHOW_DATA_
//#define _SHOW_ROTATION_
//#define _SHOW_OUTPUT_CONDITION_
//#define _SHOW_RULE_TABLE_

// MACRO CONDITION

#include    <array>

#include    <mutex>

#include    <memory>

#include    <iostream>

#include    <ros/ros.h>

#include    "read_data.cpp"

#include    <zeabus/count.hpp>

#include    <zeabus/escape_code.hpp>

#include    <tf/LinearMath/Matrix3x3.h>

#include    <zeabus_utility/ControlCommand.h>

#include    <zeabus/service/control_command.hpp>

#include    <zeabus_control/fuzzy/all_parameter.hpp>

#include    <zeabus/ros_interfaces/single_thread.hpp>

#include    <zeabus/fuzzy/control_error_3dimension.hpp>

#include    <zeabus/client/single_thread/get_auv_state.hpp>

#include    <zeabus/ros_interfaces/convert/geometry_msgs.hpp>

#include    <zeabus/client/single_thread/send_control_command.hpp>

int main( int argv , char** argc )
{

    // First part is about base code in ros system
    zeabus::ros_interfaces::SingleThread node_control_fuzzy( argv , argc , "control_fuzzy" );

    std::shared_ptr< ros::NodeHandle > ptr_node_handle =
            std::make_shared< ros::NodeHandle >("");

    ros::Publisher control_fuzzy_publisher = ptr_node_handle->advertise<zeabus_utility::ControlCommand>("/control/thruster", 1);

    std::shared_ptr< std::mutex > ptr_mutex_data = std::make_shared< std::mutex >();

    // Insert optional part param part
    const static unsigned int frequency = 5;
    const static unsigned int start_run = 0;
    zeabus_utility::ControlCommand error; // received error
    zeabus_utility::ControlCommand force; // send_force
    zeabus_utility::ControlCommand temp; // this temp for use get value from error
    zeabus_utility::AUVState current_state; // this use for get state of auv
    ros::Rate rate( frequency );
    // 2 array below will help you can skip error, mistake, not-continous data for 1 time
    std::array< bool , 6 > help_mask = { false , false , false , false , false , false };
    std::array< double , 6 > help_error = { 0 , 0 , 0 , 0 , 0 , 0 };

    // part of client for send command
    zeabus::client::single_thread::SendControlCommand client_control_fuzzy;
    client_control_fuzzy.setup_ptr_node_handle( ptr_node_handle );
    client_control_fuzzy.setup_ptr_data( &force );
    client_control_fuzzy.setup_client( "/control/thruster");
    control_fuzzy_publisher.publish(force);

    // part of client for send auv_state command
    zeabus::client::single_thread::GetAUVState client_control_state;
    client_control_state.setup_ptr_node_handle( ptr_node_handle );
    client_control_state.setup_ptr_data( &current_state );
    client_control_state.setup_client( "/fusion/auv_state" );

    // part of service
    zeabus::service::ControlCommand server_control_fuzzy;
    server_control_fuzzy.setup_ptr_node_handle( ptr_node_handle );
    server_control_fuzzy.setup_ptr_mutex_data( ptr_mutex_data );
    server_control_fuzzy.setup_ptr_data( &error );
    server_control_fuzzy.setup_server_service( "/control/fuzzy" );

    // part of fuzzy library
    zeabus::fuzzy::ControlError3Dimension fuzzy_x( ptr_node_handle, "/control/x", "odom" );
    zeabus::fuzzy::ControlError3Dimension fuzzy_y( ptr_node_handle, "/control/y", "odom" );
    zeabus::fuzzy::ControlError3Dimension fuzzy_z( ptr_node_handle, "/control/z", "odom" );
    zeabus::fuzzy::ControlError3Dimension fuzzy_roll( ptr_node_handle, "/control/roll", "odom" );
    zeabus::fuzzy::ControlError3Dimension fuzzy_pitch( ptr_node_handle, "/control/pitch","odom");
    zeabus::fuzzy::ControlError3Dimension fuzzy_yaw( ptr_node_handle, "/control/yaw" , "odom");
    zeabus::fuzzy::ControlError3Dimension* all_fuzzy[6] = {
        &fuzzy_x , &fuzzy_y , &fuzzy_z , &fuzzy_roll , &fuzzy_pitch , &fuzzy_yaw };

    // set x axis
    fuzzy_x.set_offset( zeabus_control::fuzzy::x_parameter::OFFSET );
    fuzzy_x.set_fuzzification_error( &zeabus_control::fuzzy::x_parameter::ERROR_RULE );
    fuzzy_x.set_fuzzification_diff( &zeabus_control::fuzzy::x_parameter::DIFF_RULE );
    fuzzy_x.set_fuzzification_force( &zeabus_control::fuzzy::x_parameter::FORCE_RULE );
    fuzzy_x.set_fuzzy_rule( &zeabus_control::fuzzy::MASTER_RULE );
    fuzzy_x.set_defuzzification_rule( &zeabus_control::fuzzy::x_parameter::DEFUZZY_RULE );

    // set y axis
    fuzzy_y.set_offset( zeabus_control::fuzzy::x_parameter::OFFSET );
    fuzzy_y.set_fuzzification_error( &zeabus_control::fuzzy::x_parameter::ERROR_RULE );
    fuzzy_y.set_fuzzification_diff( &zeabus_control::fuzzy::x_parameter::DIFF_RULE );
    fuzzy_y.set_fuzzification_force( &zeabus_control::fuzzy::x_parameter::FORCE_RULE );
    fuzzy_y.set_fuzzy_rule( &zeabus_control::fuzzy::MASTER_RULE );
    fuzzy_y.set_defuzzification_rule( &zeabus_control::fuzzy::x_parameter::DEFUZZY_RULE );

    // set z axis
    fuzzy_z.set_offset( zeabus_control::fuzzy::z_parameter::OFFSET );
    fuzzy_z.set_fuzzification_error( &zeabus_control::fuzzy::z_parameter::ERROR_RULE );
    fuzzy_z.set_fuzzification_diff( &zeabus_control::fuzzy::z_parameter::DIFF_RULE );
    fuzzy_z.set_fuzzification_force( &zeabus_control::fuzzy::z_parameter::FORCE_RULE );
    fuzzy_z.set_fuzzy_rule( &zeabus_control::fuzzy::MASTER_RULE );
    fuzzy_z.set_defuzzification_rule( &zeabus_control::fuzzy::z_parameter::DEFUZZY_RULE );

    // set roll axis
    fuzzy_roll.set_offset( zeabus_control::fuzzy::roll_parameter::OFFSET );
    fuzzy_roll.set_fuzzification_error( &zeabus_control::fuzzy::roll_parameter::ERROR_RULE );
    fuzzy_roll.set_fuzzification_diff( &zeabus_control::fuzzy::roll_parameter::DIFF_RULE );
    fuzzy_roll.set_fuzzification_force( &zeabus_control::fuzzy::roll_parameter::FORCE_RULE );
    fuzzy_roll.set_fuzzy_rule( &zeabus_control::fuzzy::MASTER_RULE );
    fuzzy_roll.set_defuzzification_rule( &zeabus_control::fuzzy::roll_parameter::DEFUZZY_RULE );

    // set pitch axis
    fuzzy_pitch.set_offset( zeabus_control::fuzzy::pitch_parameter::OFFSET );
    fuzzy_pitch.set_fuzzification_error( &zeabus_control::fuzzy::pitch_parameter::ERROR_RULE );
    fuzzy_pitch.set_fuzzification_diff( &zeabus_control::fuzzy::pitch_parameter::DIFF_RULE );
    fuzzy_pitch.set_fuzzification_force( &zeabus_control::fuzzy::pitch_parameter::FORCE_RULE );
    fuzzy_pitch.set_fuzzy_rule( &zeabus_control::fuzzy::MASTER_RULE );
    fuzzy_pitch.set_defuzzification_rule( &zeabus_control::fuzzy::pitch_parameter::DEFUZZY_RULE);

    // set yaw axis
    fuzzy_yaw.set_offset( zeabus_control::fuzzy::yaw_parameter::OFFSET );
    fuzzy_yaw.set_fuzzification_error( &zeabus_control::fuzzy::yaw_parameter::ERROR_RULE );
    fuzzy_yaw.set_fuzzification_diff( &zeabus_control::fuzzy::yaw_parameter::DIFF_RULE );
    fuzzy_yaw.set_fuzzification_force( &zeabus_control::fuzzy::yaw_parameter::FORCE_RULE );
    fuzzy_yaw.set_fuzzy_rule( &zeabus_control::fuzzy::MASTER_RULE );
    fuzzy_yaw.set_defuzzification_rule( &zeabus_control::fuzzy::yaw_parameter::DEFUZZY_RULE );

    node_control_fuzzy.spin();
    tf::Quaternion temp_quaternion;
    tf::Quaternion state_quaternion;
    std::cout   << "Start loop\n";
    while( ptr_node_handle->ok() )
    {
        rate.sleep();
        (client_control_state).normal_call();
        ptr_mutex_data->lock();
        temp = error;
        ptr_mutex_data->unlock();

        for( unsigned int run = 0 ; run < 6 ; run++ )
        {
            if( (temp.mask)[run] && ( ! ( (run == 3 ) || ( run == 4 ) ) ) )
            {
                (force.target)[ run ] = all_fuzzy[ run ]->push_error( (temp.target)[run] );
                (force.mask)[run] = true;
                help_mask[run] = true;
                help_error[run] = (temp.target)[run];
            }
            else if( help_mask[run] )
            {
                (force.target)[ run ] = all_fuzzy[ run ]->push_error( help_error[ run ] );
                (force.mask)[ run ] = true;
                help_mask[ run ] = false;
            }
            else
            {
                (force.mask)[ run ] = false;
                all_fuzzy[ run ]->clear_system();
            }
        }

        // we will rotation force of linear in odom frame to world frame        
        temp_quaternion = tf::Quaternion( (force.target)[0] , (force.target)[1]
                , (force.target)[2] , 0 );
        zeabus::ros_interfaces::convert::quaternion_tf(
                &(current_state.data.pose.pose.orientation)
                , &state_quaternion );
        temp_quaternion = state_quaternion.inverse() * temp_quaternion * state_quaternion;
#ifdef _SHOW_ROTATION_
        std::cout   << "force quaternion is " << temp_quaternion.x() << " " 
                    << temp_quaternion.y() << " " << temp_quaternion.z() << " " 
                    << temp_quaternion.w() << "\n";
        std::cout   << "Quaternion convert is " << state_quaternion.x() << " " 
                    << state_quaternion.y() << " " << state_quaternion.z() << " " 
                    << state_quaternion.w() << "\n";
        std::cout   << "Temp convert is " << temp_quaternion.x() << " " 
                    << temp_quaternion.y() << " " << temp_quaternion.z() << " " 
                    << temp_quaternion.w() << "\n";
#endif

#ifdef _SUMMARY_
        std::cout   << "Input\t\tMask\tOutput\t\t\tMask\n";
        for( unsigned int run = start_run ; run < 6 ; run++ )
        {
            std::cout   << temp.target[run] << "\t" << read_bool(temp.mask[run]) << "\t" 
                        << force.target[run] << "\t" << read_bool(force.mask[run]) << "\n";
        } // loop for for print summary case macro
#endif // _SUMMARY_

        (force.target)[ 0 ] = temp_quaternion.x();
        (force.target)[ 1 ] = temp_quaternion.y();
        (force.target)[ 2 ] = temp_quaternion.z();
        force.header.stamp = ros::Time::now();
        client_control_fuzzy.normal_call();
    }
    ros::shutdown();
    node_control_fuzzy.join();
    return 0;
}
