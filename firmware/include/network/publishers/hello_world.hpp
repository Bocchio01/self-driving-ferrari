#pragma once

#include <ros.h>
#include <std_msgs/String.h>
#include "network/virtuals/publisher.hpp"

// Use the F() macro to store topic name in flash memory
const char hello_world_topic[] PROGMEM = "hello_world";

class PublisherHelloWorld : public Publisher
{
private:
    ros::Publisher pub;
    std_msgs::String hello_world_msg;

public:
    PublisherHelloWorld()
        : pub(reinterpret_cast<const char *>(hello_world_topic), &hello_world_msg) {} // Using F() macro for topic name

    void init(ros::NodeHandle &nh) override
    {
        nh.advertise(pub);
    }

    void publish() override
    {
        hello_world_msg.data = reinterpret_cast<const char *>(F("Hello, World! (From Arduino via ROS)")); // Using F() macro for the message string
        pub.publish(&hello_world_msg);
    }
};
