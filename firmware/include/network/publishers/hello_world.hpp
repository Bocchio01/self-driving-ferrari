#pragma once

#include "network/ros.h"
#include <std_msgs/String.h>
#include "network/virtuals/publisher.hpp"
#include "vehicle/vehicle_interface.hpp"

class PublisherHelloWorld : public Publisher
{
private:
    ros::Publisher pub;
    std_msgs::String hello_world_msg;

public:
    PublisherHelloWorld()
        : pub("hello_world", &hello_world_msg) {}

    void init(ros::NodeHandle &nh, IVehicle &vehicle) override
    {
        this->vehicle = &vehicle;
        nh.advertise(pub);
    }

    void publish() override
    {
        hello_world_msg.data = "Hello, World! (From Arduino via ROS)";
        pub.publish(&hello_world_msg);
    }
};
