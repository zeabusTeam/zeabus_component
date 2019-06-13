// FILE			: control_fuzzy.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 27 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
// _SUMMARY_    : If you macro this, code will show you about input and output for connection
//              between interface and command thruster of control
// _SHOW_ROTATION_
//              : Will show you about rotation from output in world frame to robot frame
// --> WARNING MACRO OF FUZZY LIB
// _SHOW_DATA_  : This is macro will show you see about process of fuzzy logic and your data
// _SHOW_OUTPUT_CONDITION_
//              : This macro specific about output_condtion of fuzzy logic
// _SHOW_RULE_TABLE_
//              : This macro will alway show data of multiple index data

// README
//  Main code and process of this program is library of normal fuzzy control. Please read
//  zeabus_library_cpp/include/zeabus/fuzzy/control_error.hpp

// REFERENCE

// MACRO SET
#define _SUMMARY_
//#define _SHOW_DATA_
//#define _SHOW_ROTATION_
//#define _SHOW_OUTPUT_CONDITION_
//#define _SHOW_RULE_TABLE_

// MACRO CONDITION
#ifdef _SHOW_DATA_
    #define _SHOW_OUTPUT_CONDITION_
#endif

#include    <array>

#include    <mutex>

#include    <memory>

#include    <iostream>

#include    <ros/ros.h>

#include    "read_data.cpp"

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

    ros::Publisher control_fuzzy_publisher = ptr_node_handle->advertise<zeabus_utility::ControlCommand>("/control/thruster", 1);

    std::shared_ptr< std::mutex > ptr_mutex_data = std::make_shared< std::mutex >();

    // Insert optional part param part
    const static unsigned int frequency = 10;
    const static unsigned int start_run = 0;
    const static unsigned int size_buffer_fuzzy = 2;
    unsigned int count_loop = 0 ; // Use for make we print equal loop of fuzzy
    zeabus_utility::ControlCommand error; // received error
    zeabus_utility::ControlCommand force; // send_force
    zeabus_utility::ControlCommand temp; // this temp for use get value from error
    zeabus_utility::AUVState current_state; // this use for get state of auv
    ros::Rate rate( frequency );
    // 2 array below will help you can skip error, mistake, not-continous data for 1 time
    std::array< bool , 6 > help_mask = { false , false , false , false , false , false };
    std::array< double , 6 > help_error = { 0 , 0 , 0 , 0 , 0 , 0 };

    // Part about fuzzy logic
    //  We design two you two dimension for follow setup you should think follow this 
    //      - x is horizontal line
    //      - y is vertical line
    //      - x is range of error value -3 , -2 , -1 , 0 , 1 , 2 , 3 
    //      - y is range of different error value -3 , -2 , -1 , 0 , 1 , 2 , 3 
    std::array< std::array< int , 7 > , 7 > fuzzy_rule = {
               0 , 1  ,  3 ,  3 ,  3 , 3 , 3  
            , -1 , 0  ,  3 ,  3 ,  3 , 2 , 3
            , -1 , -1 ,  0 ,  3 ,  2 , 2 , 2
            , -2 , -1 , -1 ,  0 ,  1 , 1 , 2
            , -2 , -2 , -1 , -3 ,  0 , 1 , 1 
            , -3 , -3 , -2 , -3 , -1 , 0 , 1
            , -3 , -3 , -2 , -3 , -2 ,-3 , 0
    };

    // Can study about this object on package of zeabus_library_cpp
    zeabus::fuzzy::ControlError< size_buffer_fuzzy > fuzzy_logic[6];
    
    // Pattern is x y z roll pitch yaw
    double offset_value[6] = { 0 , 0 , -2 , 0 , 0 , 0 };
    // The next 1 type have 3 value
    double relative_value[18] = { 
            0.08 , 0.15 , 0.27   // x 
            , 0.1 , 0.2 , 0.35 // y
            , 0.04 , 0.1 , 0.2 // z
            , 0.01 , 0.05 , 0.1 // roll
            , 0.01 , 0.05 , 0.1 // pitch
            , 0.02 , 0.06 , 0.12 // yaw 
    };

    double error_range[18] = { 
            0.1 , 1.5 , 5 // x
            , 0.1 , 1.5 , 5 // y
            , 0.1 , 0.5 , 1 // z
            , 0.1 , 1 , 2  // roll
            , 0.1 , 1 , 2  // pitch
            , 0.05 , 0.5 , 1.5 // yaw
    };

    double diff_range[18] = { 
            0.05 , 0.1 , 0.2 // x
            , 0.05 , 0.1 , 0.2 // y
            , 0.05 , 0.2 , 0.5 // z
            , 0.01 , 0.15 , 0.3 // roll
            , 0.01 , 0.15 , 0.3 // pitch
            , 0.02 , 0.06 , 0.14 // yaw
    }; 

    // this force is have affect about output condition very much
    double force_range[18] = { 
            1.2 , 3 , 6.5 
            , 1.8 , 4 , 8
            , 2 , 4 , 8 
            , 0.1 , 0.3 , 0.7
            , 0.1 , 0.3 , 0.7
            , 0.05 , 0.1 , 0.2 };

    // This part we will send many data to use setup value of fuzzy logic
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
        fuzzy_logic[ run ].set_force_range( force_range[ run * 3 + 0 ] 
                , force_range[ run * 3 + 1 ] , force_range[ run * 3 + 2 ] );
        fuzzy_logic[ run ].clear_system();
    }

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
        for( unsigned int run = start_run ; run <6 ; run++ )
        {
            if( (temp.mask)[run] && ( run != 3 ) && ( run != 4 ))
            {
                (force.target)[ run ] = fuzzy_logic[ run ].push( (temp.target)[run] );
                (force.mask)[run] = true;
                help_mask[ run ] = true;
                help_error[ run ] = (temp.target)[ run ];
            }
            else if( help_mask[ run ] )
            {
                (force.target)[ run ] = fuzzy_logic[ run ].push( help_error[run] );
                (force.mask)[run] = true;
                help_mask[ run ] = false;
            }
            else
            {
#ifdef _SHOW_OUTPUT_CONDITION_
                if( count_loop == (size_buffer_fuzzy -1 ) )
                {
                    std::cout   << zeabus::escape_code::bold_red
                                << "------------------>\n"
                                << "\tWarning! You clear system\n"
                                << zeabus::escape_code::normal_white;
                }
#endif
                fuzzy_logic[ run ].clear_system();
                (force.target)[ run ] = 0;
                (force.mask)[run] = false;
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
        if( count_loop == ( size_buffer_fuzzy - 1 ) )
        {
            std::cout   << "force quaternion is " << temp_quaternion.x() << " " 
                        << temp_quaternion.y() << " " << temp_quaternion.z() << " " 
                        << temp_quaternion.w() << "\n";
            std::cout   << "Quaternion convert is " << state_quaternion.x() << " " 
                        << state_quaternion.y() << " " << state_quaternion.z() << " " 
                        << state_quaternion.w() << "\n";
            std::cout   << "Temp convert is " << temp_quaternion.x() << " " 
                        << temp_quaternion.y() << " " << temp_quaternion.z() << " " 
                        << temp_quaternion.w() << "\n";
        } 
#endif

#ifdef _SUMMARY_
        count_loop++;
        if( count_loop == size_buffer_fuzzy )
        {
            std::cout   << "Input\t\tMask\tOutput\t\t\tMask\n";
            for( unsigned int run = start_run ; run < 6 ; run++ )
            {
                std::cout   << temp.target[run] << "\t" << read_bool(temp.mask[run]) << "\t" 
                            << force.target[run] << "\t" << read_bool(force.mask[run]) << "\n";
            } // loop for for print summary case macro
            count_loop = 0;
        }
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
