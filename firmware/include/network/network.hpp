#pragma once

#include <ros.h>
#include "network/virtuals/publisher.hpp"
#include "network/virtuals/subscriber.hpp"
#include "network/virtuals/service.hpp"

#define MAX_PUBLISHERS 2  
#define MAX_SUBSCRIBERS 2  
#define MAX_SERVICES 1  

class Network {
private:
    ros::NodeHandle nh;
    VehicleCore* vehicle;

    Publisher* publishers[MAX_PUBLISHERS];
    Subscriber* subscribers[MAX_SUBSCRIBERS];
    Service* services[MAX_SERVICES];

    uint8_t pub_count = 0;
    uint8_t sub_count = 0;
    uint8_t srv_count = 0;

public:
    Network();
    Network(VehicleCore& vehicle);
    ~Network();

    void bindVehicle(VehicleCore& vehicle);
    bool addPublisher(Publisher& publisher);
    bool addSubscriber(Subscriber& subscriber);
    bool addService(Service& service);

    void init(unsigned long baud = 115200);
    void spinOnce();
};
