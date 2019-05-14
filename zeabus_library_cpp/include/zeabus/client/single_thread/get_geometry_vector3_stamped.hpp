// FILE         : get_geometry_vector3_stamped.hpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, MAY 10
// MAINTAINER   : K.Supasan

// MACRO DETAIL

// README

#include    <zeabus/client/single_thread/base_class.hpp>

#include    <zeabus_utility/GetGeometryVector3Stamped.h>

#include    <geometry_msgs/Vector3Stamped.h>

#ifndef _ZEABUS_CLIENT_SINGLE_THREAD_GET_GEOMETRY_VECTOR3_STAMPED_HPP__
#define _ZEABUS_CLIENT_SINGLE_THREAD_GET_GEOMETRY_VECTOR3_STAMPED_HPP__

namespace zeabus
{

namespace client
{

namespace single_thread
{

    class GetGeometryVector3Stamped : public BaseClass< geometry_msgs::Vector3Stamped >
    {
        public:
            GetGeometryVector3Stamped( std::shared_ptr<ros::NodeHandle> ptr_node_handle = NULL);
    
            bool setup_client( std::string topic_service );
    
            bool normal_call();

            bool thread_call();
    
            void mutex_call();

        protected:
            // have this for collecting advertise
            ros::ServiceClient client_server;

            // this will collect variable to use connect data between service and client
            zeabus_utility::GetGeometryVector3Stamped client_data;
    }:

} // namespace single_thread

} // namespace client

} // namespace zeabus

#endif
