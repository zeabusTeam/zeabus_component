// FILE			: function_helper.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, May 22 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  This file use to collect source code function
//  On this macro set will smame 

// REFERENCE

// MACRO SET
//#define _IMU_CONVERTER_

// This function use to convert NED coordinate to ENU coordinate

// This from roation yaw negate half file and then rotation roll pi in radian

void rotation_linear( const geometry_msgs::Vector3* source , geometry_msgs::Vector3* result 
        , tf::Quaternion* rotation )
{
    tf::Quaternion temp_quaternion( source->x , source->y , source->z , 0 );
    temp_quaternion = (*rotation) * temp_quaternion * ( rotation->inverse() ) ;
    result->x = temp_quaternion.x();
    result->y = temp_quaternion.y();
    result->z = temp_quaternion.z();
} // function rotation_linear

void dvl_status( bool data )
{
    static bool status = false;
    if( status )
    {
        if( !data )
        {
            ROS_WARN( "SENSOR FUSION DVL doesn't have new data"); 
            status = false;
        } // new data is false
    } // original status is true
    else
    {
        if( data )
        {
            ROS_WARN( "SENSOR FUSION DVL have new data");
            status = true; 
        } // new data is true
    } // original status is false
}

void imu_status( bool data )
{
    static bool status = false;
    if( status )
    {
        if( !data )
        {
            ROS_WARN( "SENSOR FUSION IMU doesn't have new data"); 
            status = false;
        } // new data is false
    } // original status is true
    else
    {
        if( data )
        {
            ROS_WARN( "SENSOR FUSION IMU have new data");
            status = true; 
        } // new data is true
    } // original status is false
}

void pressure_status( bool data )
{
    static bool status = false;
    if( status )
    {
        if( !data )
        {
            ROS_WARN( "SENSOR FUSION PRESSURE doesn't have new data"); 
            status = false;
        } // new data is false
    } // original status is true
    else
    {
        if( data )
        {
            ROS_WARN( "SENSOR FUSION PRESSURE have new data");
            status = true; 
        } // new data is true
    } // original status is false
}

int read_bit_value( unsigned char number )
{
    int result = 0;
    if( (number & 0b001) == 1)
    {
        result += 1;
    }
    if( (number & 0b010) == 2)
    {
        result += 2;
    }
    if( (number & 0b100) == 4)
    {
        result += 4;
    }
    return result;
}
