// FILE			: auv_state.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 22 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

#include    <zeabus/service/get_data/base_class.hpp>

#include    <zeabus_utility/AUVState.h>

#include    <zeabus_utility/GetAUVState.h>

#ifndef _ZEABUS_SERVICE_GET_SINGLE_DATA_AUV_STATE_HPP__
#define _ZEABUS_SERVICE_GET_SINGLE_DATA_AUV_STATE_HPP__

namespace zeabus
{

namespace service
{

namespace get_data
{

    class AUVState : public BaseClass< zeabis_utility::AUVState >
    {
        public:
            AUVState( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL );

        protected:
            void ensure_setup_service( std::string service_topic );

            bool callback( zeabus_utility::GetAUVState::Request& request
                    , zeabus_utility::GetAUVState::Response& response );
    } // class AUVState

} // namespace get_data

} // namespace service

} // namespace zeabus

#endif // _ZEABUS_SERVICE_GET_SINGLE_DATA_AUV_STATE_HPP__
