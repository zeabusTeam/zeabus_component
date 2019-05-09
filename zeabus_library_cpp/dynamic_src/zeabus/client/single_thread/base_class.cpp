// FILE         : base_class.cpp
// AUTHOR       : K.Supasan
// CREATE ON    : 2019, APRIL 9
// MAINTAINER   : K.Supasan

namespace zeabus
{

namespace client
{

namespace single_thread
{

    template< typename type_data >
    BaseClass< type_data >::BaseClass( std::shared_ptr< ros::NodeHandle > ptr_node_handle )
            : zeabus::client::BaseClass<1>( ptr_node_handle )
    {
        ;
    } // constructor BaseClass

    template< typename type_data >
    void BaseClass< type_data >::setup_ptr_mutex_data( 
            std::shared_ptr< std::mutex > ptr_mutex_data )
    {
        this->ptr_mutex_data = ptr_mutex_data;
    } // function setup_ptr_mutex_data

    template< typename type_data >
    void BaseClass< type_data >::setup_ptr_data( type_data* ptr_data )
    {
        this->ptr_data = ptr_data ;
    }

} // namespace single_thread

} // namespace client

} // namespace zeabus
