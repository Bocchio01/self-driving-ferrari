#pragma once

#include "network/ros.h"
#include "vehicle/vehicle_interface.hpp"

class Service
{
protected:
    IVehicle *vehicle;

public:
    virtual ~Service() {}
    virtual void init(ros::NodeHandle &nh, IVehicle &vehicle) = 0;
};
