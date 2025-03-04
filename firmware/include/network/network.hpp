#pragma once

#include "network/ros.h"
#include "network/virtuals/publisher.hpp"
#include "network/virtuals/subscriber.hpp"
#include "network/virtuals/service.hpp"
#include "vehicle/vehicle_interface.hpp"

class Network
{
private:
    ros::NodeHandle nh;
    IVehicle *vehicle;

    Publisher *publishers[MAX_PUBLISHERS];
    Subscriber *subscribers[MAX_SUBSCRIBERS];
    Service *services[MAX_SERVICES];

    uint8_t pub_count = 0;
    uint8_t sub_count = 0;
    uint8_t srv_count = 0;

public:
    Network() : vehicle(nullptr) {};
    ~Network() {};

    void bindVehicle(IVehicle &vehicle)
    {
        this->vehicle = &vehicle;
    };

    bool addPublisher(Publisher &publisher)
    {
        if (pub_count < MAX_PUBLISHERS)
        {
            publishers[pub_count++] = &publisher;
            return true;
        }
        return false;
    };

    bool addSubscriber(Subscriber &subscriber)
    {
        if (sub_count < MAX_SUBSCRIBERS)
        {
            subscribers[sub_count++] = &subscriber;
            return true;
        }
        return false;
    };

    bool addService(Service &service)
    {
        if (srv_count < MAX_SERVICES)
        {
            services[srv_count++] = &service;
            return true;
        }
        return false;
    };

    void init(unsigned long baud = 115200)
    {
        nh.getHardware()->setBaud(baud);
        nh.initNode();

        for (uint8_t i = 0; i < pub_count; i++)
        {
            publishers[i]->init(nh, *vehicle);
        }

        for (uint8_t i = 0; i < sub_count; i++)
        {
            subscribers[i]->init(nh, *vehicle);
        }

        for (uint8_t i = 0; i < srv_count; i++)
        {
            services[i]->init(nh, *vehicle);
        }
    };

    void spinOnce()
    {
        nh.spinOnce();
    }
};
