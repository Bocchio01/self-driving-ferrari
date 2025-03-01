#include <ros/ros.h>
#include "joy_to_control_cmd.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_to_control_cmd");
    JoyToControlCmd joy_to_control_cmd;

    ros::spin();
}