// FILE			: vector3_filter.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 18 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

#include    <zeabus/ros_interfaces/file/vector3_filter.hpp>

namespace zeabus
{

namespace ros_interfaces
{

namespace file
{

    Vector3Filter::Vector3Filter( std::string full_path ) : BaseClass( full_path )
    {
        ;
    } // constructor of Vector3Filter

    void Vector3Filter::write_column( std::string input_x , std::string input_y 
            , std::string input_z , std::string output_x , std::string output_y 
            , std::string output_z )
    {
        std::string message = "time," + input_x + "," + input_y + "," + input_z + ","
                + output_x + "," + output_y + "," + output_z;
        this->writeline( &message );
    } // funcction write_column

    void Vector3Filter::logging( const ros::Time* stamp , const geometry_msgs::Vector3* input
            , const geometry_msgs::Vector3* output )
    {
        std::string message = zeabus::convert::to_string( stamp->sec ) 
                + "." + zeabus::convert::to_string( stamp->nsec )
                + "," + zeabus::ros_interfaces::convert::vector3_string( input , ',' )
                + "," + zeabus::ros_interfaces::convert::vector3_string( output , ',' );
        this->writeline( &message );
    } // function collecting logging

    void Vector3Filter::logging( const ros::Time stamp , const geometry_msgs::Vector3 input
            , const geometry_msgs::Vector3 output )
    {
        (this->logging)( &stamp , &input , &output );
    } // function collecting logging

    void Vector3Filter::logging(const ros::Time* stamp,const double* input,const double* output)
    {
        std::string message = zeabus::convert::to_string( stamp->sec )
                + "." + zeabus::convert::to_string( stamp->nsec )
                + "," + zeabus::array::convert::to_string( input , 3 , ',' )
                + "," + zeabus::array::convert::to_string( output , 3 , ',' );
        this->writeline( &message );        
    }

    void Vector3Filter::logging(const ros::Time stamp,const double* input,const double* output)
    {
        (this->logging)(&stamp , input , output );
    }

} // namespace file

} // namespace ros_interfaces

} // namespace zeabus
