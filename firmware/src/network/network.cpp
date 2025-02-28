#include "network/network.hpp"
#include "vehicle/vehicle.hpp"

Network::Network() : vehicle(nullptr), pub_count(0), sub_count(0), srv_count(0) {}

Network::Network(VehicleCore& vehicle) : vehicle(&vehicle), pub_count(0), sub_count(0), srv_count(0) {}

Network::~Network() {}

void Network::bindVehicle(VehicleCore& vehicle) {
    this->vehicle = &vehicle;
}

bool Network::addPublisher(Publisher& publisher) {
    if (pub_count < MAX_PUBLISHERS) {
        publishers[pub_count++] = &publisher;
        return true;
    }
    return false;
}

bool Network::addSubscriber(Subscriber& subscriber) {
    if (sub_count < MAX_SUBSCRIBERS) {
        subscribers[sub_count++] = &subscriber;
        return true;
    }
    return false;
}

bool Network::addService(Service& service) {
    if (srv_count < MAX_SERVICES) {
        services[srv_count++] = &service;
        return true;
    }
    return false;
}

void Network::init(unsigned long baud) {
    nh.getHardware()->setBaud(baud);
    nh.initNode();

    for (uint8_t i = 0; i < pub_count; i++) publishers[i]->init(nh);
    for (uint8_t i = 0; i < sub_count; i++) subscribers[i]->init(nh, *vehicle);
    for (uint8_t i = 0; i < srv_count; i++) services[i]->init(nh, *vehicle);
}

void Network::spinOnce() {
    nh.spinOnce();
}
