// FILE         : base_class.hpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 9
// MAINTAINER   : K.Supasan

#include    <zeabus/client/base_class.hpp>

#include    <mutex>

#include    <zeabus/escape_code.hpp>

// README
//  Please remember this baseclass will help you manage thread, mutex and data single object.
//  You don't worry about setup mutex and data

// MACRO DETAIL

#ifndef _ZEABUS_CLIENT_SINGLE_THREAD_BASE_CLASS_HPP__
#define _ZEABUS_CLIENT_SINGLE_THREAD_BASE_CLASS_HPP__

namespace zeabus
{

namespace client
{

namespace single_thread
{

    template< typename type_data >
    class BaseClass : public zeabus::client::BaseClass< 1 >
    {
        public:
            BaseClass( std::shared_ptr< ros::NodeHandle > ptr_node_handle = NULL );
            
            void setup_ptr_mutex_data( std::shared_ptr< std::mutex > ptr_mutex_data );

            void setup_ptr_data( type_data* ptr_data );

        protected:
            std::shared_ptr< std::mutex > ptr_mutex_data;
            type_data* ptr_data;

    }; // class BaseClass

} // namespace single_thread

} // namespace client

} // namespace zeabus

#include    <zeabus/client/single_thread/base_class.cpp>

#endif // _ZEABUS_CLIENT_SINGLE_THREAD_BASE_CLASS_HPP__
