#include <zeabus/fuzzy/control_velocity_output.hpp>

#include <zeabus_utility/ControlCommand.h>

#include "ros/ros.h"

#include <iostream>

double input[6];
bool check_mask[6];
// double output[6];
zeabus_utility::ControlCommand output;

void callback( const zeabus_utility::ControlCommand&  msg)
{
    for(unsigned int run = 0 ; run < 6 ; run++ )
    {
        check_mask[run] = msg.mask[run];
        if(msg.mask[run])
        {
            input[run] = msg.target[run];
        }
        else
        {
            input[run] = 0;
        }
        std::cout << "input " << run << ":" << " " << input[run];
        if(check_mask[run])
        {
            std::cout << "  mask: True" << std::endl;
        }
        else
        {
            std::cout << "  mask: False" << std::endl;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_by_force");
    ros::NodeHandle sub;
    ros::NodeHandle pub;
    ros::Subscriber control_subscriber = sub.subscribe("/control/fuzzy", 2 , callback);
    ros::Publisher control_publisher = pub.advertise<zeabus_utility::ControlCommand>("/control/thrusters", 1000);
    ros::Rate rate(10);
    while(sub.ok())
    {
        rate.sleep();
        ros::spinOnce();
        for(unsigned int run = 0 ; run < 6 ; run++)
        {
            if(run == 2 || run == 5 || run == 0 || run == 1)
            {
                output.target[run] = run_system(input[run] ,run, check_mask[run]);
                output.mask[run] = true;
            }
        }
        control_publisher.publish(output);
    }
}
