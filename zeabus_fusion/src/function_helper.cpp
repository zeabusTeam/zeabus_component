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

const static tf::Quaternion quaternion_coordinate = tf::Quaternion( 
        0.707 , -0.707 , 0 , 0 );

void NED_to_ENU( tf::Quaternion* data )
{
    double RPY[3];
#ifdef _IMU_CONVERTER_
    std::cout   << "NED quaternion    : " 
                << data->w() << " " << data->x() << " " << data->y() << " " << data->z() << "\n";
#endif // _IMU_CONVERTER_

#ifdef _IMU_CONVERTER_
    tf::Matrix3x3( *data ).getRPY( RPY[0] , RPY[1] , RPY[2] );
    std::cout   << "Euler NED : " 
                << RPY[0] << " " << RPY[1] << " " << RPY[2] << "\n";
#endif // _IMU_CONVERTER_
    *data = quaternion_coordinate*(*data);
#ifdef _IMU_CONVERTER_
    std::cout   << "ENU quaternion    : " 
                << data->w() << " " << data->x() << " " << data->y() << " " << data->z() << "\n";
#endif // _IMU_CONVERTER_
    tf::Matrix3x3( *data ).getRPY( RPY[0] , RPY[1] , RPY[2] );
#ifdef _IMU_CONVERTER_
    std::cout   << "Euler ENU form before offset : " 
                << RPY[0] << " " << RPY[1] << " " << RPY[2] << "\n";
#endif
    RPY[0] += zeabus::radian::negate_pi;
    RPY[2] += zeabus::radian::negate_half_pi;
    zeabus::radian::bound( &(RPY[0]) );
    zeabus::radian::bound( &(RPY[2]) );
    data->setRPY( RPY[0] , RPY[1] , RPY[2] );
#ifdef _IMU_CONVERTER_
    std::cout   << "Euler ENU form after offset  : " 
                << RPY[0] << " " << RPY[1] << " " << RPY[2] << "\n";
#endif
#ifdef _IMU_CONVERTER_
    std::cout   << "RESULT quaternion : " 
                << data->w() << " " << data->x() << " " << data->y() << " " << data->z() << "\n";
#endif
} // function NED_to_ENU

void rotation_linear( const geometry_msgs::Vector3* source , geometry_msgs::Vector3* result 
        , tf::Quaternion* rotation )
{
    tf::Quaternion temp_quaternion( source->x , source->y , source->z , 0 );
    temp_quaternion = (*rotation) * temp_quaternion * ( rotation->inverse() ) ;
    result->x = temp_quaternion.x();
    result->y = temp_quaternion.y();
    result->z = temp_quaternion.z();
} // function rotation_linear
