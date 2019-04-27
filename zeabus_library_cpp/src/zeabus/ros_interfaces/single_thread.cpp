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
        ros::init( argv , argc , node_name );
        this->status_thread = false;
        this->node_name = node_name;
        this->node_handle = ros::NodeHandle("");
    } // function constructor object SingleThread parameter prefix_name

    bool SingleThread::spin()
    {
        bool result = false;
        if( (this->node_handle).ok() )
        {
            if( this->status_thread )
            {
                this->thread_id = std::thread( std::bind( 
                        &zeabus::ros_interfaces::SingleThread::thread_spin , this )
                ); // declare or init start thread
                result = true;
            } // that mean you ever run or finish run spin
            else
            {
                std::cout   << "Now thread are spinning\n";
            }
        } // that mean ros node are ok
        else
        {
            std::cout   << "Node handle doesn't ok\n";
        }
        return result;
    } // function SingleThread::spin

    bool SingleThread::status()
    {
        return this->status_thread;
    }

    void SingleThread::thread_spin()
    {
        if( (this->node_handle).ok() )
        {
            this->status_thread = true;
            std::cout   << node_name << "start spin thread\n";
            ros::spin();
            std::cout   << node_name << "end spin thread\n";
            this->status_thread = false;
        }
        else
        {
            std::cout   << "Can't spin node handle doesn't ok\n";
        }
    } // function SingleThread::thread_spin()

} // namespace ros_interfaces

} // namespace zeabus
