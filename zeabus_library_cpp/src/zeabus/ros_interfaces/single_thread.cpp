// FILE         : single_thread.cpp
// AUTHOR       : Supasan Komonlit
// CREATE ON    : 2019 , APRIL 28
// MAINTAINER   : Supasan Komonlit

#include    <zeabus/ros_interfaces/single_thread.hpp>

namespace zeabus
{

namespace ros_interfaces
{
    SingleThread::SingleThread( int argv , char** argc , std::string node_name  )  
    {
        std::cout   << "ros::init function\n";
        ros::init( argv , argc , node_name );
        this->status_thread = false;
        this->node_name = node_name;
        std::cout   << "create node handle\n";
    } // function constructor object SingleThread parameter prefix_name

    SingleThread::~SingleThread()
    {
        std::cout   << "desconstructor SingleThread\n";
        ros::shutdown();
    } // function desconstructor objeect SingleThread

    bool SingleThread::spin()
    {
        bool result = false;
        if( ! this->status_thread )
        {
            this->thread_id = std::thread( std::bind( 
                    &zeabus::ros_interfaces::SingleThread::thread_spin , this )
            ); // declare or init start thread
            result = true;
        } // that mean you ever run or finish run spin
        else
        {
            std::cout   << "Thread spining already spin and active now\n";
        }
        return result;
    } // function SingleThread::spin

    bool SingleThread::status()
    {
        return this->status_thread;
    }

    void SingleThread::thread_spin()
    {
        this->status_thread = true;
        std::cout   << zeabus::escape_code::normal_red << node_name 
                    << " start spin thread\n" << zeabus::escape_code::normal_white;
        ros::spin();
        std::cout   << zeabus::escape_code::bold_red << node_name 
                    << " end spin thread\n" << zeabus::escape_code::normal_white;
        this->status_thread = false;
    } // function SingleThread::thread_spin()

} // namespace ros_interfaces

} // namespace zeabus
