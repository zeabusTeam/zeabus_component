// FILE         : pololu.hpp 
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 30
// MAINTAINER   : Supasan Komonlit

#include    <zeabus/service/base_class.hpp>

#include    <zeabus_utility/SendUInt16Array.h>

#include    <vector>
// Because we must push data in to vector type unsigned short int 16 bits

// MACRO_DETAIL
//  _SENDER_DETAIL_     --> This macro will print detail about sender when callback been call
//  _CALLBACK_CALLED_   --> Print when callback have been called

#ifdef _SENDER_DETAIL_
    #define _CALLBACK_CALLED_
#endif // _SENDER_DETAIL_

#ifndef _ZEABUS_SERVICE_POLOLU_HPP__
#define _ZEABUS_SERVICE_POLOLU_HPP__

namespace zeabus
{

namespace service
{

    class Pololu : public zeabus::service::BaseClass
    {
        public:
            Pololu( std::shared_ptr< ros::NodeHandle > ptr_node_handle );

            bool setup_server_service( std::string service_topic );

            // setup ptr data have 2 arguments to use
            //  First is ptr_buffer is pointer to buffer
            //  Seconde is size of buffer you should to received
            void setup_ptr_data( std::vector< unsigned short int >* ptr_buffer 
                    , unsigned int size_target , ros::Time* ptr_time_updated );

        protected:
            bool callback( zeabus_utility::SendUInt16Array::Request& request
                    , zeabus_utility::SendUInt16Array::Response& response );

            bool already_setup_ptr_data;

            ros::ServiceServer server_service;

        private:
            std::vector< unsigned short int >* ptr_buffer;
            unsigned int size_target;
            ros::Time* ptr_time_updated;
        
    }; // class Pololu

} // namespace service

} // namespace zeabus

#endif
