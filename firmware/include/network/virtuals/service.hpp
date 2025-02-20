#pragma once



#include <ros.h>
#include "vehicle/vehicle.hpp"

class Service {
public:
    virtual ~Service() {}
    virtual void init(ros::NodeHandle& nh, VehicleCore& vehicle) = 0;
};
