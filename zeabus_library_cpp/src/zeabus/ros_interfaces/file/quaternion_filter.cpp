// FILE			: quaternion_filter.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 21 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

#include    <zeabus/ros_interfaces/file/quaternion_filter.hpp>

namespace zeabus
{

namespace ros_interfaces
{

namespace file
{

    QuaternionFilter::QuaternionFilter( std::string full_path ) : BaseClass( full_path )
    {
        ;
    } // constructor of QuaternionFilter

    void QuaternionFilter::write_column( std::string input_w , std::string input_x
            , std::string input_y , std::string input_z , std::string output_w 
            , std::string output_x , std::string output_y , std::string output_z )
    {
        std::string message = "time," + input_w + "," + input_x + "," + input_y + "," + input_z
                + "," + output_w + "," + output_x + "," + output_y + "," +output_z;
        this->writeline( &message );
    }

    void Vector3Filter::logging( const ros::Time* stamp , const geometry_msgs::Quaternion* input
            , const geometry_msgs::Quaternion* output )
    {
        std::string message = zeabus::convert::to_string( stamp->sec )
                + "." + zeabus::convert::to_string( stamp->nsec )
                + "," + zeabus::ros_interfaces::convert::quaternion_filter( input , ',' )
                + "," + zeabus::ros_interfaces::convert::quaternion_filter( output , ',');
    }

    void Vector3Filter::logging( const ros::Time stamp , const geometry_msgs::Quaternion input
            , const geometry_msgs::Quaternion output )
    {
        this->logging( &stamp , &input , &output );
    }

} // namespace file

} // namespace ros_interfaces

} // namespace zeabus
