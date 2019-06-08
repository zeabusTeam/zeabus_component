// FILE			: raw_fusion.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, June 08 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
// This log type prepare for fusion path with 3 input and single output is AUVState

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <iostream>

#include    <zeabus/ros_interfaces/file/base_class.hpp>

#include    <ros/ros.h>

#include    <zeabus/convert/to_string.hpp>

#include    <zeabus_utility/AUVState.h>

#ifndef _ZEABUS_ROS_INTERFACES_FILE_RAW_FUSION_HPP__
#define _ZEABUS_ROS_INTERFACES_FILE_RAW_FUSION_HPP__

namespace zeabus
{

namespace ros_interfaces
{

namespace file
{

    class RawFusion : public BaseClass
    {
        public:
            RawFusion( std::string full_path = "" );

            // We automatic write time for first paragraph
            void write_cloumn( std::string velocity_x = "dvl_vel_x" 
                , std::string velocity_y = "dvl_vel_y" 
                , std::string velocity_z = "dvl_vel_z" 
                , std::string orieantation_x = "imu_orieantation_x" 
                , std::string orieantation_y = "imu_orieantation_y"
                , std::string orieantation_z = "imu_orieantation_z" 
                , std::string orieantation_w = "imu_orieantation_w" 
                , std::string gyro_x = "imu_gyro_x"
                , std::string gyro_y = "imu_gyro_y"
                , std::string gyro_z = "imu_gyro_z"
                , std::string pressure_pose_z = "pressure_pose_z"
                , std::string output_pose_x = "output_pose_x"
                , std::string output_pose_x = "output_pose_y"
                , std::string output_pose_x = "output_pose_z"
                , std::string output_oreantation_x = "output_oreantation_x"
                , std::string output_oreantation_y = "output_oreantation_y"
                , std::string output_oreantation_z = "output_oreantation_z"
                , std::string output_oreantation_w = "output_oreantation_w"
                , std::string output_linear_x = "output_vel_x"
                , std::string output_linear_y = "output_vel_y"
                , std::string output_linear_z = "output_vel_z"
                , std::string output_gyro_x = "output_gyro_x"
                , std::string output_gyro_y = "output_gyro_y"
                , std::string output_gyro_z = "output_gyro_x" )

            void logging( zeabus_utility::AUVState* data );
    }

} // namespace file

} // namespace ros_interfaces

} // namespace zeabus

#endif // _ZEABUS_ROS_INTERFACES_FILE_RAW_FUSION_HPP__
