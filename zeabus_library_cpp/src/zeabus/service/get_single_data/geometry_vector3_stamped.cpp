// FILE         : geometry_vector3_stamped.cpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 29
// MAINTAINER   : Supasan Komonlit

#include    <zeabus/service/get_single_data/geometry_vector3_stamped.hpp>

namespace zeabus
{

namespace service
{

namespace get_single_data
{

    GeometryVector3Stamped::GeometryVector3Stamped( 
            std::shared_ptr< ros::NodeHandle > ptr_node_handle , std::string frame_id) 
            : BaseClass( ptr_node_handle , frame_id )
    {
        ; // nothing to do in constructor fo subclass
    } //  Constructor GeometryVector3Stamped

    void GeometryVector3Stamped::ensure_setup_service( std::string service_topic )
    {
        this->service_server = this->ptr_node_handle->advertiseService( service_topic 
                , &zeabus::service::get_single_data::GeometryVector3Stamped::callback , this );
    } // function ensure_setup_service

    bool GeometryVector3Stamped::callback( 
            zeabus_utility::GetGeometryVector3Stamped::Request& request 
            , zeabus_utility::GetGeometryVector3Stamped::Response& response )
    {
        this->ptr_mutex_data->lock();
        response.data = *(this->ptr_data);
        this->ptr_mutex_data->unlock();
        response.header.stamp = ros::Time();
        response.header.frame_id = this->frame_id; 
        return true;
    } // function callback

} // namespace get_single_data

} // namespace service

} // namespace zeabus
