// FILE			: header_float64.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 13 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

#include    <zeabus/service/get_data/base_class.hpp>

#include    <zeabus_utility/DepthCommand.h>

#include    <zeabus_utility/HeaderFloat64.h>

#ifndef _ZEABUS_SERVICE_GET_SINGLE_DATA_HEADER_FLOAT64_HPP
#define _ZEABUS_SERVICE_GET_SINGLE_DATA_HEADER_FLOAT64_HPP

namespace zeabus
{

namespace service
{

namespace get_data
{

    class HeaderFloat64 : public BaseClass< zeabus_utility::HeaderFloat64 >
    {

        public:
            HeaderFloat64( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL ); 

        protected:
            void ensure_setup_service( std::string service_topic );

            bool callback( zeabus_utility::DepthCommand::Request& request 
                    , zeabus_utility::DepthCommand::Response& response );

    }; // class HeaderFloat64

} // namespace get_data

} // namespace service

} // namespace zeabus

#endif
