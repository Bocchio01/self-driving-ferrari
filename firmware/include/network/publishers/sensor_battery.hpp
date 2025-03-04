#pragma once

#include <std_msgs/UInt8.h>
#include "network/ros.h"
#include "network/virtuals/publisher.hpp"
#include "vehicle/vehicle_interface.hpp"

class PublisherHelloWorld : public Publisher
{
private:
    IKinematic *vehicle;
    ros::Publisher pub;

    std_msgs::UInt8 battery_msg;

public:
    PublisherHelloWorld()
        : pub("sensor_battery", &battery_msg) {}

    void init(ros::NodeHandle &nh, Vehicle &vehicle) override
    {
        this->vehicle = &vehicle;
        nh.advertise(pub);
    }

    void publish() override
    {
        // if (this->vehicle && this->vehicle->sensors && this->vehicle->sensors->battery)
        // {
        //     battery_msg.data = this->vehicle->sensors->battery->read();
        //     pub.publish(&battery_msg);
        // }
    }
};
