// FILE			: raw_fusion.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, June 08 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <zeabus/ros_interfaces/file/raw_fusion.hpp>

namespace zeabus
{

namespace ros_interfaces
{

namespace file
{

    RawFusion::RawFusion( std::string full_path ) : BaseClass( full_path )
    {
        ;
    } // function constructure of RawFusion

    void RawFusion::write_column( std::string velocity_x  , std::string velocity_y
        , std::string velocity_z , std::string orieantation_x , std::string orieantation_y 
        , std::string orieantation_z, std::string orieantation_w, std::string gyro_x
        , std::string gyro_y, std::string gyro_z, std::string pressure_pose_z
        , std::string output_pose_x, std::string output_pose_y, std::string output_pose_z
        , std::string output_oreantation_x, std::string output_oreantation_y
        , std::string output_oreantation_z, std::string output_oreantation_w 
        , std::string output_linear_x, std::string output_linear_y, std::string output_linear_z
        , std::string output_gyro_x, std::string output_gyro_y, std::string output_gyro_z 
        , std::string output_status )
    {
        std::string message = "time," + velocity_x 
            + "," + velocity_y 
            + "," + velocity_z
            + "," + orieantation_x 
            + "," + orieantation_y 
            + "," + orieantation_z
            + "," + orieantation_w 
            + "," + gyro_x 
            + "," + gyro_y 
            + "," + gyro_z  
            + "," + pressure_pose_z  
            + "," + output_pose_x  
            + "," + output_pose_y  
            + "," + output_pose_z  
            + "," + output_oreantation_x  
            + "," + output_oreantation_y  
            + "," + output_oreantation_z  
            + "," + output_oreantation_w  
            + "," + output_linear_x  
            + "," + output_linear_y  
            + "," + output_linear_z  
            + "," + output_gyro_x  
            + "," + output_gyro_y  
            + "," + output_gyro_z
            + "," + output_status;
        this->writeline( &message );  
    } // function write column

    void RawFusion::setup_ptr_dvl_data( geometry_msgs::Vector3* ptr_data )
    {
        this->ptr_dvl_data = ptr_data;
    } // function setup_ptr_dvl_data

    void RawFusion::setup_ptr_imu_data( sensor_msgs::Imu* ptr_data )
    {
        this->ptr_imu_data = ptr_data;
    } // function setup_ptr_imu_data

    void RawFusion::setup_ptr_pressure_data( double* ptr_data )
    {
        this->ptr_pressure_data = ptr_data;
    }

    void RawFusion::logging( zeabus_utility::AUVState* data )
    {
        std::string message = zeabus::convert::to_string( data->data.header.stamp.secs)
            + "." + zeabus::convert::to_string( data->data.header.stamp.nsec )
            + "," + zeabus::convert::to_string( this->ptr_dvl_data->x )
            + "," + zeabus::convert::to_string( this->ptr_dvl_data->y )
            + "," + zeabus::convert::to_string( this->ptr_dvl_data->z )
            + "," + zeabus::convert::to_string( this->ptr_imu_data->orientation.x )
            + "," + zeabus::convert::to_string( this->ptr_imu_data->orientation.y )
            + "," + zeabus::convert::to_string( this->ptr_imu_data->orientation.z )
            + "," + zeabus::convert::to_string( this->ptr_imu_data->orientation.w )
            + "," + zeabus::convert::to_string( this->ptr_imu_data->angular_velocity.x )
            + "," + zeabus::convert::to_string( this->ptr_imu_data->angular_velocity.y )
            + "," + zeabus::convert::to_string( this->ptr_imu_data->angular_velocity.z )
            + "," + zeabus::convert::to_string( *(this->ptr_pressure_data) )
            + "," + zeabus::convert::to_string( this->data->pose.pose.position.x )
            + "," + zeabus::convert::to_string( this->data->pose.pose.position.y )
            + "," + zeabus::convert::to_string( this->data->pose.pose.position.z )
            + "," + zeabus::convert::to_string( this->data->pose.pose.orientation.x )
            + "," + zeabus::convert::to_string( this->data->pose.pose.orientation.y )
            + "," + zeabus::convert::to_string( this->data->pose.pose.orientation.z )
            + "," + zeabus::convert::to_string( this->data->pose.pose.orientation.w )
            + "," + zeabus::convert::to_string( this->data->twist.twist.linear.x )
            + "," + zeabus::convert::to_string( this->data->twist.twist.linear.y )
            + "," + zeabus::convert::to_string( this->data->twist.twist.linear.z )
            + "," + zeabus::convert::to_string( this->data->twist.twist.angular.x )
            + "," + zeabus::convert::to_string( this->data->twist.twist.angular.y )
            + "," + zeabus::convert::to_string( this->data->twist.twist.angular.z )
            + "," + zeabus::convert::to_string( this->data->status );
        this->writeline( &message );
    } // function logging

} // namespace file

} // namespace ros_interfaces

} // namespace zeabus
