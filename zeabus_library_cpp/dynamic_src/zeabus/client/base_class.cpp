// FILE         : base_class.cpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019, APRIL 8
// MAINTAINER   : K.Supasan

namespace zeabus
{

namespace client
{
    
    template<unsigned int size_thread>
    BaseClass< size_thread >::BaseClass( std::shared_ptr< ros::NodeHandle > ptr_node_handle )
    {
        this->already_setup_ptr_node_handle = false;
        if( ptr_node_handle != NULL )
        {
            this->setup_ptr_node_handle( ptr_node_handle );
        }
        for( unsigned int run = 0 ; run < size_thread ; run++ )
        {
            (this->thread_status)[run] = false;
        }
    } // function constructor

    template<unsigned int size_thread>
    void BaseClass< size_thread >::setup_ptr_node_handle( 
            std::shared_ptr< ros::NodeHandle > ptr_node_handle )
    {
        this->ptr_node_handle = ptr_node_handle;
        this->already_setup_ptr_node_handle = true;
    } // function setup_ptr_node_handle

    template<unsigned int size_thread>
    void BaseClass< size_thread >::thread_join()
    {
        for( unsigned int run = 0 ; run < size_thread ; run++ )
        {
            (this->thread_id)[ run ].join();
#ifdef _PRINT_JOIN_PROCESS_
            std::cout   << "Joining thread_id " << run << std::endl;
#endif // _PRINT_JOIN_PROCESS_
        }
    } // function thread_join

} // namespace client

} // namespace zeabus
