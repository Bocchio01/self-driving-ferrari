#include <ros/ros.h>
#include "vehicle_cmd_gate.hpp"
#include "ferrari_common/control_cmd.h"
#include "ferrari_common/set_gate_mode.h"
#include "ferrari_common/get_gate_mode.h"

VehicleCmdGate::VehicleCmdGate()
    : gate_mode(GateMode::INTERNAL)
{
    this->sub_internal_control_cmd = nh.subscribe("internal_control_cmd", 1, &VehicleCmdGate::internalControlCmdCallback, this);
    this->sub_external_control_cmd = nh.subscribe("external_control_cmd", 1, &VehicleCmdGate::externalControlCmdCallback, this);
    this->pub_control_cmd = nh.advertise<ferrari_common::control_cmd>("control_cmd", 1);
    this->srv_set_gate_mode = nh.advertiseService("set_gate_mode", &VehicleCmdGate::setGateModeCallback, this);
    this->srv_get_gate_mode = nh.advertiseService("get_gate_mode", &VehicleCmdGate::getGateModeCallback, this);
}

VehicleCmdGate::~VehicleCmdGate()
{
}

void VehicleCmdGate::internalControlCmdCallback(const ferrari_common::control_cmd::ConstPtr &msg)
{
    if (this->gate_mode == GateMode::INTERNAL)
    {
        sendControlCmd(msg);
    }
}

void VehicleCmdGate::externalControlCmdCallback(const ferrari_common::control_cmd::ConstPtr &msg)
{
    if (this->gate_mode == GateMode::EXTERNAL)
    {
        sendControlCmd(msg);
    }
}

void VehicleCmdGate::sendControlCmd(const ferrari_common::control_cmd::ConstPtr &msg)
{
    this->pub_control_cmd.publish(msg);
}

bool VehicleCmdGate::setGateModeCallback(ferrari_common::set_gate_mode::Request &req,
                                         ferrari_common::set_gate_mode::Response &res)
{
    if (req.gate_mode != static_cast<int>(GateMode::INTERNAL) &&
        req.gate_mode != static_cast<int>(GateMode::EXTERNAL))
    {
        ROS_ERROR("Invalid gate mode: %d", req.gate_mode);
        res.success = false;
        return false;
    }

    this->gate_mode = static_cast<GateMode>(req.gate_mode);
    res.success = true;
    return true;
}

bool VehicleCmdGate::getGateModeCallback(ferrari_common::get_gate_mode::Request &req,
                                         ferrari_common::get_gate_mode::Response &res)
{
    res.gate_mode = static_cast<int>(this->gate_mode);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_cmd_gate");
    VehicleCmdGate vehicle_cmd_gate;
    ros::spin();
    return 0;
}