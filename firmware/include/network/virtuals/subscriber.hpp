#pragma once

#include "network/ros.h"
#include "vehicle/vehicle_interface.hpp"

class Subscriber
{
protected:
    IVehicle *vehicle;

public:
    virtual ~Subscriber() {}
    virtual void init(ros::NodeHandle &nh, IVehicle &vehicle) = 0;
};
