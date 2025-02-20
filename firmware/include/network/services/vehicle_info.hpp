#pragma once



#include "network/virtuals/service.hpp"
#include "vehicle/vehicle.hpp"
#include <ros.h>
#include <std_msgs/String.h>

class VehicleInfoService : public Service
{
private:
    ros::ServiceServer<std_msgs::String, VehicleInfoService> server;
    VehicleCore *vehicle;

    static bool callback(std_msgs::String::Request &req, std_msgs::String::Response &res, VehicleInfoService *instance)
    {
        return instance->handleRequest(req, res);
    }

public:
    VehicleInfoService() : server("get_kinematics", &VehicleInfoService::callback, this) {}

    void init(ros::NodeHandle &nh, VehicleCore &v) override
    {
        vehicle = &v;
        nh.advertiseService(server);
    }

    bool handleRequest(std_msgs::String::Request &req, std_msgs::String::Response &res)
    {
        if (!vehicle)
            return false;
        res.data = vehicle->getKinematicsType();
        return true;
    }
};
