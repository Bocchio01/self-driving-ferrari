#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "ferrari_common/control_cmd.h"
#include "joy_to_control_cmd.hpp"

JoyToControlCmd::JoyToControlCmd()
{
    this->sub_joy = nh.subscribe("/joy", 1, &JoyToControlCmd::joyCallback, this);
    this->pub_external_control_cmd = nh.advertise<ferrari_common::control_cmd>("external_control_cmd", 1);
}

JoyToControlCmd::~JoyToControlCmd()
{
}

void JoyToControlCmd::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    ferrari_common::control_cmd control_cmd;

    control_cmd.header.stamp = ros::Time::now();
    control_cmd.steering_angle = joy->axes[0];
    control_cmd.speed = joy->axes[1];

    this->pub_external_control_cmd.publish(control_cmd);
}