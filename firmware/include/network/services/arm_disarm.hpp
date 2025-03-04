#pragma once

#include "network/ros.h"
#include "vehicle/vehicle_interface.hpp"
#include "network/virtuals/service.hpp"
#include "ferrari_common/toggle_arm_vehicle.h"

class ServiceArmDisarm : public Service
{
private:
    ros::ServiceServer<ferrari_common::toggle_arm_vehicle::Request,
                       ferrari_common::toggle_arm_vehicle::Response,
                       ServiceArmDisarm>
        srv;

    void handleServiceRequest(const ferrari_common::toggle_arm_vehicle::Request &req,
                              ferrari_common::toggle_arm_vehicle::Response &res)
    {
        this->vehicle->isArmed() ? this->vehicle->executeDisarming() : this->vehicle->executeArming();

        res.success = true;
        res.is_armed = this->vehicle->isArmed();
    }

public:
    ServiceArmDisarm()
        : srv("toggle_arm_vehicle", &ServiceArmDisarm::handleServiceRequest, this) {}

    void init(ros::NodeHandle &nh, IVehicle &vehicle) override
    {
        this->vehicle = &vehicle;
        nh.advertiseService(srv);
    }
};
