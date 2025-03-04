#pragma once

#include "network/ros.h"
#include "vehicle/vehicle_interface.hpp"

class Publisher
{
protected:
    IVehicle *vehicle;

public:
    virtual ~Publisher() {}
    virtual void init(ros::NodeHandle &nh, IVehicle &vehicle) = 0;
    virtual void publish() = 0;
};