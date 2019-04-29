// FILE         : geometry_vector3_stamped.hpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 29
// MAINTAINER   : Supasan Komonlit

#include    <zeabus/service/get_single_data/base_class.hpp>

#include    <zeabus_utility/GetGeometryVector3Stamped.h>

#include    <geometry_msgs/Vector3Stamped.h>

#ifndef _ZEABUS_SERVICE_GET_SINGLE_DATA_GEOMETRY_VECTOR3_STAMPED_HPP
#define _ZEABUS_SERVICE_GET_SINGLE_DATA_GEOMETRY_VECTOR3_STAMPED_HPP

namespace zeabus
{

namespace service
{

namespace get_single_data
{

    class GeometryVector3Stamped : public BaseClass< geometry_msgs::Vector3Stamped >
    {
        public:
            SensorImu( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL 
                    , std::string frame_id = "dvl" );

        protected:

            void ensure_setup_service( std::string service_topic );

            bool callback( zeabus_utility::GetGeometryVector3Stamped::Request& request 
                    , zeabus_utility::GetGeometryVector3Stamped::Response& response );

    }; // class GeometryVector3Stamped

} // namespace get_single_data

} // namespace service

} // namespace zeabus

#endif // _ZEABUS_SERVICE_GET_SINGLE_DATA_GEOMETRY_VECTOR3_STAMPED_HPP
