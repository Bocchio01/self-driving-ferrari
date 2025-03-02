#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "kinematic_adapter_interface.hpp"
#include "ferrari_common/control_cmd.h"
#include "ferrari_common/get_kinematic_model.h"

class VehicleAdapter
{
protected:
    ros::NodeHandle nh;
    ros::ServiceClient srv_get_kinematic_model;

    KinematicAdapterInterface *kinematic_adapter;

    void queryVehicleKinematicModel();

public:
    VehicleAdapter();
    ~VehicleAdapter();
};