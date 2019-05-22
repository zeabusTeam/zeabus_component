// FILE			: header_float64.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 13 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

#include    <zeabus/service/get_data/header_float64.hpp>

namespace zeabus
{

namespace service
{

namespace get_data
{

    HeaderFloat64::HeaderFloat64( std::shared_ptr< ros::NodeHandle > ptr_node_handle 
            , std::string frame_id ) : BaseClass( ptr_node_handle , frame_id )
    {
        ; 
    } // constructor HeaderFloat64

    void HeaderFloat64::ensure_setup_service( std::string service_topic )
    {
        this->service_server = this->ptr_node_handle->advertiseService( service_topic 
                , &zeabus::service::get_data::HeaderFloat64::callback , this );
    }

    bool HeaderFloat64::callback( zeabus_utility::DepthCommand::Request& request
            , zeabus_utility::DepthCommand::Response& response )
    {
        this->ptr_mutex_data->lock();
        response.header = this->ptr_data->header;
        response.depth = this->ptr_data->data;
        this->ptr_mutex_data->unlock();
        return true;
    }

} // namespace get_data

} // namespace service

} // namespace zeabus
