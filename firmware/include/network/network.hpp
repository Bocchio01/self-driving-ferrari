#pragma once

#include <ros.h>
#include <StandardCplusplus.h>

#include <vector>
#include <memory>

#include "network/virtuals/publisher.hpp"
#include "network/virtuals/subscriber.hpp"
#include "network/virtuals/service.hpp"

class Network {
private:
    ros::NodeHandle nh;
    VehicleCore* vehicle;

    std::vector<Publisher*> publishers;
    std::vector<Subscriber*> subscribers;
    std::vector<Service*> services;

public:
    Network();
    Network(VehicleCore& vehicle);
    ~Network();

    void bindVehicle(VehicleCore& vehicle);

    void addPublisher(Publisher& publisher);
    void addSubscriber(Subscriber& subscriber);
    void addService(Service& service);

    void init();
    void spinOnce();
};
