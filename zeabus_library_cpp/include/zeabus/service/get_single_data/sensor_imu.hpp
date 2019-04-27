// FILE         : sensor_imu.hpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 27
// MAINTAINER   : Supasan Komonlit

#include    <zeabus/service/get_single_data/base_class.hpp>

#include    <sensor_msgs/Imu.h>

#ifndef _ZEABUS_SERVICE_GET_SINGLE_DATA_SENSOR_IMU_HPP
#define _ZEABUS_SERVICE_GET_SINGLE_DATA_SENSOR_IMU_HPP

namespace zeabus
{

namespace service
{

namespace get_single_data
{

    class SensorImu : public BaseClass< sensor_msgs::Imu >
    {
        public:
            SensorImu( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL );

        protected:
        
            void ensure_setup_service( std::string service_topic );

            void callback( const sensor_msgs::Imu& message );

    }; // class SensorImu

} // namespace get_single_data

} // namespace service

} // namespace zeabus

#endif // _ZEABUS_SERVICE_GET_SINGLE_DATA_SENSOR_IMU_HPP
