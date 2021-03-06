// FILE         : geometry_vector3_stamped.cpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 29
// MAINTAINER   : Supasan Komonlit

#include    <zeabus/service/get_data/geometry_vector3_stamped.hpp>

namespace zeabus
{

namespace service
{

namespace get_data
{

    GeometryVector3Stamped::GeometryVector3Stamped( 
            std::shared_ptr< ros::NodeHandle > ptr_node_handle ) : BaseClass( ptr_node_handle)
    {
        ; // nothing to do in constructor fo subclass
    } //  Constructor GeometryVector3Stamped

    void GeometryVector3Stamped::ensure_setup_service( std::string service_topic )
    {
        this->service_server = this->ptr_node_handle->advertiseService( service_topic 
                , &zeabus::service::get_data::GeometryVector3Stamped::callback , this );
    } // function ensure_setup_service

    bool GeometryVector3Stamped::callback( 
            zeabus_utility::GetGeometryVector3Stamped::Request& request 
            , zeabus_utility::GetGeometryVector3Stamped::Response& response )
    {
        this->ptr_mutex_data->lock();
        response.data = *(this->ptr_data);
        this->ptr_mutex_data->unlock();
        return true;
    } // function callback

} // namespace get_data

} // namespace service

} // namespace zeabus
