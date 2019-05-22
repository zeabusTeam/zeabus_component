// FILE			: fusion_3_thread.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This header will collect command to call 3 service in real time. What that mean?
//  I will split thread to 3 thread to call all data 

// REFERENCE

// MACRO SET

#include    <mutex>

#include    <memory>

#include    <iostream>

#include    <ros/ros.h>

#include    <sensor_msgs/Imu.h>

#include    <zeabus_utility/HeaderFloat64.h>

#include    <zeabus/client/base_class.hpp>

#include    <geometry_msgs/Vector3Stamped.h>

#include    <zeabus/client/single_thread/get_sensor_imu.hpp>

#include    <zeabus/client/single_thread/get_depth_command.hpp>

#include    <zeabus/client/single_thread/get_geometry_vector3_stamped.hpp>

#ifndef _ZEABUS_CLIENT_FUSION_3_THREAD_HPP__
#define _ZEABUS_CLIENT_FUSION_3_THREAD_HPP__

namespace zeabus
{

namespace client
{

    class Fusion3Thread : public zeabus::client::BaseClass< 3 >
    {
        public:
            Fusion3Thread( std::shared_ptr< ros::NodeHandle > ptr_node_handle );

            void setup_ptr_mutex_data( std::shared_ptr< std::mutex > dvl_mutex 
                    , std::shared_ptr< std::mutex > imu_mutex 
                    , std::shared_ptr< std::mutex > pressure_mutex );

            void setup_client( std::string* dvl_topic , std::string* imu_topic 
                    , std::string* pressure_topic );

            void setup_all_data( geometry_msgs::Vector3Stamped* dvl_data 
                    , sensor_msgs::Imu* imu_data ,zeabus_utility::HeaderFloat64* pressure_data );

            void all_call();

        protected:
            zeabus::client::single_thread::GetDepthCommand pressure_client;
            zeabus::client::single_thread::GetGeometryVector3Stamped dvl_client;
            zeabus::client::single_thread::GetSensorImu imu_client;
    };

} // namespce client

} // namespace zeabus

#endif // _ZEABUS_CLIENT_FUSION_3_THREAD_HPP__
