#pragma once



#include <ros.h>
#include <std_msgs/String.h>
#include "network/virtuals/publisher.hpp"

class HelloWorldPublisher : public Publisher  {
private:
    ros::Publisher pub;
    std_msgs::String hello_world_msg;

public:
    HelloWorldPublisher() : pub("hello_world", &hello_world_msg) {}

    void init(ros::NodeHandle& nh) override {
        nh.advertise(pub);
    }

    void publish() override {
        pub.publish(&hello_world_msg);
    }
};
