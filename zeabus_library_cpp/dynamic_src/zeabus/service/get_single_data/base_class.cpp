// FILE         : base_class.cpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 28
// MAINTAINER   : Supasan Komonlit

namespace zeabus
{

namespace service
{

namespace get_single_data
{

    template< class data_type >
    BaseClass< data_type >::BaseClass( std::shared_ptr< ros::NodeHandle > ptr_node_handle 
            , std::string frame_id )
    {
        this->already_setup_ptr_node_handle = false;
        this->already_setup_ptr_mutex_data = false;
        this->already_setup_ptr_data = false;
        if( ptr_node_handle != NULL )
        {
            this->setup_ptr_node_handle( ptr_node_handle );
        }
        this->setup_frame_id( frame_id );
    } //  function constructor BaseClass

    template< class data_type >
    void BaseClass< data_type >::setup_frame_id( std::string frame_id )
    {
        this->frame_id = frame_id;
    } // function setup_frame_id

    template< class data_type >
    void BaseClass<data_type>::setup_ptr_node_handle( 
            std::shared_ptr< ros::NodeHandle > ptr_node_handle )
    {
        this->ptr_node_handle = ptr_node_handle;
        this->already_setup_ptr_node_handle = true;
    } // function setup_ptr_node_handle 

    template< class data_type >
    void BaseClass<data_type>::setup_ptr_mutex_data(
            std::shared_ptr< std::mutex > ptr_mutex_data )
    {
        this->ptr_mutex_data = ptr_mutex_data;
        this->already_setup_ptr_mutex_data = true;
    }
    
    template< class data_type >
    void BaseClass<data_type>::register_data( data_type* ptr_data )
    {
        this->ptr_data = ptr_data;
        this->already_setup_ptr_data = true;
    } // function register_data

    template< class data_type >
    bool BaseClass<data_type>::setup_server_service( std::string service_topic )
    {
        bool result = this->check_setup_service();
        if( result )
        {
            this->ensure_setup_service( service_topic );
        } // that mean you can set up
        return result;
    } // function setup_server_service

    template< class data_type >
    bool BaseClass<data_type>::check_setup_service()
    {
        unsigned int count_case = 0;
        bool result = false;
        if( ros::isInitialized() )
        {
            count_case++;
        }
        else
        {
            std::cout   << "This process doesn't use ros::init for api NodeHandle\n";
        }

        if( this->already_setup_ptr_data )
        {
            count_case++;
        }
        else
        {
            std::cout   << "Please setup ptr data\n";
        }

        if( this->already_setup_ptr_mutex_data )
        {
            count_case++;
        }
        else
        {
            std::cout   << "Please setup ptr mutex data\n";
        }

        if( this->already_setup_ptr_node_handle )
        {
            count_case++;
        }
        else
        {
            std::cout   << "Please setup ptr node handle\n";
        }
        
        if( count_case == 4 )
        {
            result = true;
        }

        return result;
    } // function check_setup_service
    
} // namespace get_single_data

} // namespace service

} // namespace zeabus
