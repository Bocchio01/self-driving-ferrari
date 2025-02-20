#pragma once

#include <ros.h>
#include <vector>
#include <memory>

#include "network/virtuals/publisher.hpp"
#include "network/virtuals/subscriber.hpp"
#include "network/virtuals/service.hpp"

class Network {
private:
    ros::NodeHandle nh;
    VehicleCore* vehicle;

    std::vector<std::shared_ptr<Publisher>> publishers;
    std::vector<std::shared_ptr<Subscriber>> subscribers;
    std::vector<std::shared_ptr<Service>> services;

public:
    Network();
    Network(VehicleCore& vehicle);
    ~Network();

    void bindVehicle(VehicleCore& vehicle);

    void addPublisher(std::shared_ptr<Publisher> publisher);
    void addSubscriber(std::shared_ptr<Subscriber> subscriber);
    void addService(std::shared_ptr<Service> service);

    void init();
    void spinOnce();
};
