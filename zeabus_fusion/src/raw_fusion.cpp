// FILE			: raw_fusion.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
//  _IMU_CONVERTER_ : This macro use in file function_helper.cpp about convert value from imu.
//                  That about convert coordinate from NED to ENU
//  _ROBOT_DATA_    : This mean will show current data of robot by all sensor include position
//  _RAW_DATA_      : This macro will show you raw data to get from filter
//  _PROCESS_       : This macro use you can know about process of this code

// README
//  This will you data from 3 node to fusion but use only simple equation to fusion data
//  This node response only calculating data If have same time over this file will stop to send
//  new data to every node
//  For send data state we will don't use object class we mangae on this code

// REFERENCE

// MACRO SET
#define _IMU_CONVERTER_
#define _ROBOT_DATA_
//#define _PROCESS_

// MACRO CONDITION
#ifdef _RAW_DATA_
    #undef _ROBOT_DATA_
#endif

#include    <thread>

#include    <memory>

#include    <iostream>

#include    <ros/ros.h>

#include    <zeabus/radian.hpp>

#include    <nav_msgs/Odometry.h>

#include    <zeabus/escape_code.hpp>

#include    <geometry_msgs/Vector3.h>

#include    <tf/LinearMath/Matrix3x3.h>

#define TF_EULER_DEFAULT_ZYX
#include    <tf/LinearMath/Quaternion.h>

#include    <zeabus/convert/to_string.hpp>

#include    <zeabus/ros_interfaces/time.hpp>

#include    <zeabus/client/fusion_3_thread.hpp>

#include    <zeabus/service/get_data/auv_state.hpp>

#include    <zeabus/ros_interfaces/single_thread.hpp>

#include    <zeabus/ros_interfaces/convert/geometry_msgs.hpp>

void NED_to_ENU( tf::Quaternion* data );
void rotation_linear( const geometry_msgs::Vector3* source , geometry_msgs::Vector3* result 
        , tf::Quaternion* rotation );

#include    "function_helper.cpp"

int main( int argv , char** argc )
{
    // First part is about base file in ros system
    zeabus::ros_interfaces::SingleThread node_sensor_fusion( argv , argc , "sensor_fusion" );

    std::shared_ptr< ros::NodeHandle > ptr_node_handle = 
            std::make_shared< ros::NodeHandle >("");

    std::shared_ptr< std::mutex > dvl_mutex = std::make_shared< std::mutex >();
    std::shared_ptr< std::mutex > imu_mutex = std::make_shared< std::mutex >();
    std::shared_ptr< std::mutex > pressure_mutex = std::make_shared< std::mutex >();
    std::shared_ptr< std::mutex > ptr_mutex_data = std::make_shared< std::mutex >();
    // last variable will use for Odometry but we use about problem shared variable
    //  One Read many write but we don't worry about that because now our system can
    //  use thread for server only one.

    // Insert optional part param part
    static signed int frequency = 30;
    static std::string dvl_topic = "/filter/dvl";
    static std::string imu_topic = "/sensor/imu";
    static std::string pressure_topic = "/filter/pressure";
    static double temp_RPY[3] = { 0 , 0 , zeabus::radian::negate_half_pi };
    static tf::Quaternion offset_quaternion;;
    offset_quaternion.setRPY( temp_RPY[0] , temp_RPY[1] , temp_RPY[2] );

    // Second part of variable to use in this pid
    static zeabus_utility::HeaderFloat64 pressure_data;
    static sensor_msgs::Imu imu_data;
    static geometry_msgs::Vector3Stamped dvl_data;
    static ros::Time dvl_stamp = ros::Time::now();
    static ros::Time imu_stamp = ros::Time::now();
    static ros::Time pressure_stamp = ros::Time::now();
    static unsigned char status_data = 0b000;
    static zeabus_utility::AUVState service_data; // this use in server we will lock this
    static zeabus_utility::AUVState temp_data;

    // Third part of client get data
    zeabus::client::Fusion3Thread client_data( ptr_node_handle );
    client_data.setup_all_data( &dvl_data , &imu_data , &pressure_data );
    client_data.setup_ptr_mutex_data( dvl_mutex , imu_mutex , pressure_mutex );
    client_data.setup_client( &dvl_topic , &imu_topic , &pressure_topic );

    // Forth part of server data
    zeabus::service::get_data::AUVState server_state( ptr_node_handle );
    server_state.setup_ptr_mutex_data( ptr_mutex_data );
    server_state.register_data( &service_data );
    server_state.setup_server_service( "/fusion/auv_state");

    // Fifth part of about state of robot
    //  Purpose of this calculate that is for calculate distance from start poitn
    //  We must intergral 3 data for 2 data ( x position and y position )
    //  Don't worry about depth because that is absolute data
    static geometry_msgs::Vector3 robot_distance;
    static geometry_msgs::Vector3 world_distance;
    // In the future we will function to reset this
    static unsigned int count_dvl = 5;
    static tf::Quaternion temp_quaternion;
    // But first time is only about dvl time_stamp

    ros::Rate rate( frequency );
    // Init data first time
    service_data.data.header.frame_id = "robot";
    service_data.data.twist.twist.linear.x = 0;
    service_data.data.twist.twist.linear.y = 0;
    service_data.data.twist.twist.linear.z = 0;
    temp_data.data.pose.pose.position.x = 0;
    temp_data.data.pose.pose.position.y = 0;
    node_sensor_fusion.spin();
    while( ptr_node_handle->ok() )
    {
#ifdef _PROCESS_
        std::cout   << "Time start call : " << zeabus::ros_interfaces::time::string() << "\n";
#endif
        client_data.all_call(); 
        client_data.thread_join();
#ifdef _PROCESS_
        std::cout   << "Time end call : " << zeabus::ros_interfaces::time::string() << "\n";
#endif
#ifdef _RAW_DATA_
        std::cout   << "DVL data time : " << dvl_data.header.stamp.sec << "."
                    << dvl_data.header.stamp.nsec << "\n"
                    << "x : " << dvl_data.vector.x << " y : " << dvl_data.vector.y
                    << " z : " << dvl_data.vector.z << "\n";
        std::cout   << "IMU data time : " << imu_data.header.stamp.sec << "."
                    << imu_data.header.stamp.nsec << "\n"
                    << "x : " << imu_data.orientation.x << " y : " << imu_data.orientation.y
                    << " z : " << imu_data.orientation.z << " w : " << imu_data.orientation.z 
                    << "\n";
        std::cout   << "DEPTH data time : " << pressure_data.header.stamp.sec << "."
                    << pressure_data.header.stamp.nsec << " data is "
                    << pressure_data.data << "\n";
#endif // _RAW_DATA_
        // This part will check about time stamp and set status of message
        status_data = 0b111; // start at every data is ok
        // DVL CHECK
        if( dvl_stamp != dvl_data.header.stamp )
        {
            dvl_stamp = dvl_data.header.stamp;
            temp_data.data.twist.twist.linear.x = dvl_data.vector.x/1000;
            temp_data.data.twist.twist.linear.y = dvl_data.vector.y/1000;
            temp_data.data.twist.twist.linear.z = dvl_data.vector.z/1000;
#ifdef _PROCESS_
            std::cout   << "Update linear_velocity!\n";
#endif
        }
        else
        {
            count_dvl++;
        }
        if( imu_stamp != imu_data.header.stamp )
        {
            imu_stamp = imu_data.header.stamp;
            zeabus::ros_interfaces::convert::vector3_quaternion( &(imu_data.angular_velocity ) 
                    , &temp_quaternion );
            temp_quaternion = offset_quaternion *temp_quaternion* (offset_quaternion.inverse());
            zeabus::ros_interfaces::convert::quaternion_vector3( 
                    &temp_quaternion , &(temp_data.data.twist.twist.angular) );
#ifdef _PROCESS_
            std::cout   << "Update angular_velocity!\n";
#endif
        }
        else
        {
            status_data &= 0b101; 
        }
        if( pressure_stamp != pressure_data.header.stamp )
        {
#ifdef _PROCESS_
            std::cout   << "Update DEPTH!\n";
#endif
            temp_data.data.pose.pose.position.z = (pressure_data.data * -1);
        }
        else
        {
            status_data &= 0b011;
        }
        // Next we will rotation imu data
        if( (status_data & 0b010) != 0 )
        {
            zeabus::ros_interfaces::convert::quaternion_tf( &(imu_data.orientation) 
                    , &temp_quaternion );
            NED_to_ENU( &temp_quaternion ); 
            temp_quaternion = offset_quaternion * temp_quaternion ;
#ifdef _ROBOT_DATA_
            tf::Matrix3x3( temp_quaternion ).getRPY( temp_RPY[0], temp_RPY[1], temp_RPY[2] );
            std::cout   << "ROBOT Euler " << "Roll : " << temp_RPY[0]
                        << " Pitch : " << temp_RPY[1] << " Yaw : " << temp_RPY[2] << "\n";
#endif
            // data orientation of robot
#ifdef _PROCESS_
            std::cout   << "Update orientation!\n";
#endif
            zeabus::ros_interfaces::convert::tf_quaternion( &temp_quaternion 
                    , &(temp_data.data.pose.pose.orientation ) );
            if( count_dvl > 5 )
            {
                std::cout   << zeabus::escape_code::bold_red << "FATAL! DVL Lose data\n"
                            << zeabus::escape_code::normal_white;     
                status_data &= 0b110;
            }
            else
            {
                robot_distance.x = ( service_data.data.twist.twist.linear.x 
                        + temp_data.data.twist.twist.linear.x ) / ( frequency * 2 );
                robot_distance.y = ( service_data.data.twist.twist.linear.y 
                        + temp_data.data.twist.twist.linear.y ) / ( frequency * 2 );
                rotation_linear( &robot_distance , &world_distance , &temp_quaternion );
                temp_data.data.pose.pose.position.x += world_distance.x;
                temp_data.data.pose.pose.position.y += world_distance.y;
#ifdef _PROCESS_
                std::cout   << "Update position\n";
#endif
#ifdef _ROBOT_DATA_
                std::cout   << "robot_distance x : y <---> " << robot_distance.x << " : "
                            << robot_distance.y << "\nworld_distance x: y <---> " 
                            << world_distance.x << " : " << world_distance.y << std::endl;
#endif // _ROBOT_DATA_ 
            }
        }
#ifdef _ROBOT_DATA_
        std::cout   << "ROBOT Position < x , y , z > : < " 
                    << temp_data.data.pose.pose.position.x << " , "
                    << temp_data.data.pose.pose.position.y << " , "
                    << temp_data.data.pose.pose.position.z << " >\n";   
        std::cout   << "ROBOT State " << status_data << "\n";
#endif // _ROBOT_DATA_
        ptr_mutex_data->lock();
        service_data.data.header.stamp = ros::Time::now();
        service_data.data.pose = temp_data.data.pose;
        service_data.data.twist = temp_data.data.twist;
        service_data.status = status_data;
        ptr_mutex_data->unlock();
        rate.sleep();        
    }
    ros::shutdown();
    node_sensor_fusion.join();

}
