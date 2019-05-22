// FILE			: geometry_msgs.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 19 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

#include    <zeabus/ros_interfaces/convert/geometry_msgs.hpp>

namespace zeabus
{

namespace ros_interfaces
{

namespace convert
{

    std::string vector3_string( const geometry_msgs::Vector3* data, const char delim )
    {
        return zeabus::convert::to_string( data->x ) + delim
                + zeabus::convert::to_string( data->y ) + delim
                + zeabus::convert::to_string( data->z );
    } // function vector3_string

    std::string vector3_string( const geometry_msgs::Vector3 data, const char delim )
    {
        return vector3_string( &data , delim );
    } // function vector3_string

    std::string quaternion_string( const geometry_msgs::Quaternion* data, const char delim )
    {
        return zeabus::convert::to_string( data->w ) + delim
                + zeabus::convert::to_string( data->x ) + delim
                + zeabus::convert::to_string( data->y ) + delim
                + zeabus::convert::to_string( data->z );
    }

    std::string quaternion_string( const geometry_msgs::Quaternion data , const char delim )
    {
        return quaternion_string( &data , delim );
    }

    void tf_quaternion( const tf::Quaternion* source , geometry_msgs::Quaternion* target )
    {
        target->w = (source->w)();
        target->x = (source->x)();
        target->y = (source->y)();
        target->z = (source->z)();
    }

    void quaternion_tf( const geometry_msgs::Quaternion* source , tf::Quaternion* target )
    {
        *target = tf::Quaternion( source->x , source->y , source->z , source->w );
    }

} // namespace convert

} // namespace ros_interfaces

} // namespace zeabus
