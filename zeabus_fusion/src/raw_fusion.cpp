// FILE			: raw_fusion.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This will you data from 3 node to fusion but use only simple equation to fusion data
//  This node response only calculating data If have same time over this file will stop to send
//  new data to every node
//  For send data state we will don't use object class we mangae on this code

// REFERENCE

// MACRO SET

#include    <thread>

#include    <memory>

#include    <iostream>

#include    <ros/ros.h>

#include    <nav_msgs/Odometry.h>

#include    <tf/LinearMath/Quaternion.h>

#include    <zeabus/client/fusion_3_thread.hpp>

#include    <zeabus/ros_interfaces/single_thread.hpp>

int main( int argv , char** argc )
{
    // First part is about base file in ros system
    zeabus::ros_interfaces::SingleThread node_sensor_fusion( argv , argc , "sensor_fusion" );

    std::shared_ptr< ros::NodeHandle > ptr_node_handle = 
            std::make_shared< ros::NodeHandle >("");

    std::shared_ptr< std::mutex > dvl_mutex = std::make_shared< std::mutex >();
    std::shared_ptr< std::mutex > imu_mutex = std::make_shared< std::mutex >();
    std::shared_ptr< std::mutex > pressure_mutex = std::make_shared< std::mutex >();
    std::shared_ptr< std::mutex > ptr_data_mutex = std::make_shared< std::mutex >();
    // last variable will use for Odometry but we use about problem shared variable
    //  One Read many write but we don't worry about that because now our system can
    //  use thread for server only one.

    // Insert optional part param part
    const static unsigned int limit_time = 5; // this rule will use to all data
    const static unsigned int frequency = 30;
    const static std::string dvl_topic = "/filter/dvl";
    const static std::string imu_topic = "/filter/imu";
    const static std::string pressure_topic = "/filter/pressure";

    // Second part of variable to use in this pid
    static zeabus_utility::HeaderFloat64 pressure_data;
    static sensor_msgs::Imu imu_data;
    static geometry_msgs::Vector3Stammped dvl_data;
    static nav_msgs::Odometry state_data; // this use in temp collect data 
    static ros::Time dvl_stamp = ros::Time::now();
    static ros::Time imu_stamp = ros::Time::now();
    static ros::Time pressure_stamp  ros::Time::now();
    static unsigned char status_data = 0b000;
    zeabus_utility::AUVState service_data; // this use in server we will lock this

    // Third part of client get data
    zeabus::client::Fusion3Thread client_data( ptr_node_handle );
    client_data.setup_all_data( &dvl_data , &imu_data , &pressure_data );
    client_data.setup_ptr_mutex_data( dvl_mutex , imu_mutex , pressure_mutex );
    client_data.setup_client( dvl_topic , imu_topic , pressure_topic );

    // Forth part of server data
    

}
