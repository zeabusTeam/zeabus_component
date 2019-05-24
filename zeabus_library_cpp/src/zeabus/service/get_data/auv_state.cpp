// FILE			: auv_state.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 22 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

#include    <zeabus/service/get_data/auv_state.hpp>

namespace zeabus
{

namespace service
{

namespace get_data
{

    AUVState::AUVState( std::shared_ptr< ros::NodeHandle > ptr_node_handle )
            : BaseClass( ptr_node_handle )
    {
        ;
    } // constructor AUVState

    void AUVState::ensure_setup_service( std::string service_topic )
    {
        this->service_server = this->ptr_node_handle->advertiseService( service_topic
                , &zeabus::service::get_data::AUVState::callback , this );
    } // function ensure_setup_service

    bool AUVState::callback( zeabus_utility::GetAUVState::Request& request
            , zeabus_utility::GetAUVState::Response& response )
    {
        this->ptr_mutex_data->lock();
        response.auv_state = *(this->ptr_data);
        this->ptr_mutex_data->unlock();
        return true;
    } // function callback

} // namespace get_data

} // namespace service

} // namespace zeabus
