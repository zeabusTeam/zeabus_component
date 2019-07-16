#include <zeabus/fuzzy/control_by_velocity.hpp>

#include <zeabus_utility/ControlCommand.h>

#include "ros/ros.h"

#include <iostream>

double input[6];
// double output[6];
zeabus_utility::ControlCommand output;

void callback( const zeabus_utility::ControlCommand&  msg)
{
    for(unsigned int run = 0 ; run < 6 ; run++ )
    {
        if( msg.mask[run] )
        {
            input[ run ] = msg.target[run];
        }
        else
        {
            input[ run ] = 0;
        }
        std::cout << "input" << input[run] << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_by_force");
    ros::NodeHandle sub;
    ros::NodeHandle pub;
    ros::Subscriber control_subscriber = sub.subscribe("/control/fuzzy", 1000, callback);
    ros::Publisher control_publisher = pub.advertise<zeabus_utility::ControlCommand>("/control/thrusters", 1000);
    ros::Rate rate( 10 );
    while( sub.ok() )
    {
        rate.sleep();
        ros::spinOnce();
        for( unsigned int run = 0 ; run < 6 ; run++ )
        {
            if(run == 2)
            {
                output.target[ run ] = run_system( input[ run ] ,run);
                output.mask[ run ] = true;
            }
        }
        control_publisher.publish(output);
    }
}
