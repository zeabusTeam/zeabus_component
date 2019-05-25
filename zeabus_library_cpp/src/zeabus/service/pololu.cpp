// FILE         : pololu.cpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, MAY 2
// MAINTAINER   : Supasan Komonlit

#include    <zeabus/service/pololu.hpp>

namespace zeabus
{

namespace service
{

    Pololu::Pololu( std::shared_ptr< ros::NodeHandle > ptr_node_handle ) 
            : BaseClass( ptr_node_handle )
    {
        this->already_setup_ptr_data = false;
    } // constructor of object Pololu

    void Pololu::setup_ptr_data( std::vector< unsigned short int >* ptr_buffer , 
            unsigned int size_target , ros::Time* ptr_time_updated )
    {
        this->ptr_buffer = ptr_buffer;
        this->size_target = size_target;
        this->already_setup_ptr_data = true;
        this->ptr_time_updated = ptr_time_updated;
    } // function setup_ptr_data

    bool Pololu::setup_server_service( std::string service_topic )
    {
        bool result = true;
        if( ! (this->already_setup_ptr_data ) )
        {
            std::cout   << zeabus::escape_code::bold_red 
                        << "Service of pololu can't setup please setup_ptr_data\n"
                        << zeabus::escape_code::normal_white;
            result = false;
        }
        if( ! ( this->already_setup_ptr_mutex_data ) )
        {
            std::cout   << zeabus::escape_code::bold_red
                        << "Service of pololu can't setup please setup_ptr_mutex_data\n"
                        << zeabus::escape_code::normal_white;
            result = false;
        }
        if( ! ( this->already_setup_ptr_node_handle ) )
        {
            std::cout   << zeabus::escape_code::bold_red
                        << "Service of pololu can't setup please setup_ptr_node_handle\n"
                        << zeabus::escape_code::normal_white;
            result = false;
        }
        if( result )
        {
            this->server_service = this->ptr_node_handle->advertiseService( service_topic 
                , &zeabus::service::Pololu::callback , this );
        }
        return result;
    } // function setup_server_service

    bool Pololu::callback( zeabus_utility::SendThrottle::Request& request
            , zeabus_utility::SendThrottle::Response& response )
    {
#ifdef _CALLBACK_CALLED_
        std::cout   << "callback of pololu have been called\n";
    #ifdef _SENDER_DETAIL_
        std::cout   << "Sender at time " << request.header.stamp.sec << "."
                    << request.header.stamp.nsec << "\n";
    #endif // _SENDER_DETAIL_
#endif // _CALLBACK_CALLED_
        this->ptr_mutex_data->lock();
        this->ptr_buffer->clear();
        for( unsigned int run = 0 ; run < this->size_target ; run++ )
        {
            this->ptr_buffer->push_back( (request.data)[run] );
        }
        *(this->ptr_time_updated) = ros::Time::now();
        this->ptr_mutex_data->unlock();
#ifdef _SENDER_DETAIL_
        ros::Time temp_time = ros::Time::now();
        std::cout   << "End service at time " << temp_time.sec << "." << temp_time.nsec << "\n";
#endif // _SENDER_DETAIL_
        return true;
    } // function callback

} // namespace service

} // namespace zeabus
