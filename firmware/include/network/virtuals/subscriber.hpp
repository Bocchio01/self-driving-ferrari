#pragma once

#include <ros.h>
#include "vehicle/vehicle.hpp"

class Subscriber {
public:
    virtual ~Subscriber() {}
    // virtual void init(ros::NodeHandle& nh) = 0;
    virtual void init(ros::NodeHandle& nh, VehicleCore& vehicle) = 0;
};
