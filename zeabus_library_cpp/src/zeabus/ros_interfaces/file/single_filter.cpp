// FILE			: single_filter.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 11 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

#include <zeabus/ros_interfaces/file/single_filter.hpp>

namespace zeabus
{

namespace ros_interfaces
{

namespace file
{

    SingleFilter::SingleFilter( std::string full_path ) : BaseClass( full_path )
    {
        ;
    }

    void SingleFilter::write_column( std::string input_name , std::string output_name )
    {
        std::string message = "time," + input_name + "," + output_name;
        this->writeline( &message );
    }

    void SingleFilter::logging( ros::Time* stamp , double* input , double* output )
    {
        std::string message = "";
        message += zeabus::convert::to_string( stamp->sec ) + "." 
                    + zeabus::convert::to_string( stamp->nsec ) + ","
                    + zeabus::convert::to_string( *input ) + ","
                    + zeabus::convert::to_string( *output );
        this->writeline( &message );
    }

    void SingleFilter::logging( ros::Time stamp , double input , double output )
    {
        this->logging( &stamp , &input , &output );
    }

} // namespace file

} // namespace ros_interfaces

} // namespace zeabus
