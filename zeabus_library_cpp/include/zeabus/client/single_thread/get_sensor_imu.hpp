// FILE         : get_sensor_imu.hpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

// MACRO DETAIL
// _CALL_PROCESS_       : Use this to see when we arein function call

// MACRO SET
//#define _CALL_PROCESS_


#include    <zeabus/client/single_thread/base_class.hpp>

#include    <zeabus_utility/GetSensorImu.h>

#include    <sensor_msgs/Imu.h>

#ifndef _ZEABUS_CLIENT_SINGLE_THREAD_GET_SENSOR_IMU_HPP__
#define _ZEABUS_CLIENT_SINGLE_THREAD_GET_SENSOR_IMU_HPP__

namespace zeabus
{

namespace client
{

namespace single_thread
{

    class GetSensorImu : public zeabus::client::single_thread::BaseClass< sensor_msgs::Imu >
    {

        public:
            GetSensorImu( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL);

            bool setup_client( std::string topic_service );
 
            bool normal_call(); // call by sequence not using thread is

            bool thread_call(); // use thread id to call service

            void mutex_call();

        protected:
            ros::ServiceClient client_server;
            zeabus_utility::GetSensorImu client_data;

    }; // class GetSensorImu

} // namespace single_thread

} // namespace client

} // namespace zeabus

#endif
