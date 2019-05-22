// FILE         : sensor_imu.hpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 27
// MAINTAINER   : Supasan Komonlit

#include    <zeabus/service/get_data/base_class.hpp>

#include    <zeabus_utility/GetSensorImu.h>

#include    <sensor_msgs/Imu.h>

#ifndef _ZEABUS_SERVICE_GET_SINGLE_DATA_SENSOR_IMU_HPP
#define _ZEABUS_SERVICE_GET_SINGLE_DATA_SENSOR_IMU_HPP

namespace zeabus
{

namespace service
{

namespace get_data
{

    class SensorImu : public BaseClass< sensor_msgs::Imu >
    {
        public:
            SensorImu( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL ); 

        protected:

            void ensure_setup_service( std::string service_topic );

            bool callback( zeabus_utility::GetSensorImu::Request& request 
                    , zeabus_utility::GetSensorImu::Response& response );

    }; // class SensorImu

} // namespace get_data

} // namespace service

} // namespace zeabus

#endif // _ZEABUS_SERVICE_GET_SINGLE_DATA_SENSOR_IMU_HPP
