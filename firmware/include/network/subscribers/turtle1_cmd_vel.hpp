#pragma once

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "vehicle/vehicle.hpp"
#include "network/virtuals/subscriber.hpp"

const char turtle_cmd_vel_topic[] PROGMEM = "turtle1/cmd_vel";

class SubscriberTurtle1CmdVel : public Subscriber
{
private:
    VehicleCore *vehicle;
    ros::Subscriber<geometry_msgs::Twist, SubscriberTurtle1CmdVel> sub;

    void cmdCallback(const geometry_msgs::Twist &msg)
    {
        this->vehicle->move(msg.angular.z, msg.linear.x);
    }

public:
    SubscriberTurtle1CmdVel()
        : vehicle(nullptr),
          sub([]
              { static char topic_buffer[20]; strcpy_P(topic_buffer, turtle_cmd_vel_topic); return topic_buffer; }(),
              &SubscriberTurtle1CmdVel::cmdCallback,
              this) {}

    void init(ros::NodeHandle &nh, VehicleCore &vehicle) override
    {
        this->vehicle = &vehicle;
        nh.subscribe(sub);
    }
};
