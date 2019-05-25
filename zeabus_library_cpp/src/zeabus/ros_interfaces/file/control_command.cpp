// FILE			: control_command.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 25 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MARCO CONDITION

#include    <zeabus/ros_interfaces/file/control_command.hpp>

namespace zeabus
{

namespace ros_interfaces
{

namespace file
{

    ControlCommand::ControlCommand( std::string full_path ) : BaseClass( full_path )
    {
        ;
    } // function constructure of ControlCommand

    void ControlCommand::write_column( std::string* num_part , std::string* bool_part )
    {
        std::string message = "time";
        for( unsigned int run = 0 ; run < 6 ; run++ )
        {
            message += "," + (*num_part)[run];
        }
        for( unsigned int run = 0 ; run < 6 ; run++ )
        {
            message += "," + (*bool_part)[run];
        }
        this->writeline( &message );
    } // function write_column

    void ControlCommand::logging( const ros::Time* stamp , const std::vector< double >* num_part
            , const std::vector< uint8_t >* bool_part )
    {
        std::string message = zeabus::convert::to_string( stamp->sec ) 
                + "." + zeabus::convert::to_string( stamp->nsec );
        for( unsigned int run = 0 ; run < 6 ; run++ )
        {
            message += "," + zeabus::convert::to_string( (*num_part)[ run ] );
        }
        for( unsigned int run = 0 ; run < 6 ; run++ )
        {
            message += "," + zeabus::convert::to_string( (*bool_part)[ run ] );
        }
        this->writeline( &message );
    } // function logging by ptr

    void ControlCommand::logging( const ros::Time stamp, const std::vector< double > num_part
            , const std::vector< uint8_t > bool_part )
    {
        this->logging( &stamp , &num_part , &bool_part );
    } // function logging by data

} // namespace file

} // namespace ros_interfaces

} // namespace zeabus
