#include <zeabus/fuzzy/control_by_force.hpp>

#include "ros/ros.h"

void callback(double msg)
{
    ROS_INFO("", msg->);
}

int main(int argv, char** argc)
{
    ros::init(argc, argv, "control_by_force");
    ros::NodeHandle control;
    ros::Subscriber control_subscriber = control.subscribe("", 1000, callback);
    ros::spin();
}