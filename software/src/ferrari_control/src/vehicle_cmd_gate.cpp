#include <ros/ros.h>
#include "vehicle_cmd_gate.hpp"
#include "ferrari_common/control_cmd.h"
#include "ferrari_common/set_gate_mode.h"
#include "ferrari_common/is_engage.h"

VehicleCmdGate::VehicleCmdGate()
    : is_engaged(false),
      gate_mode(GateMode::INTERNAL)
{
    this->sub_internal_control_cmd = nh.subscribe("internal_control_cmd", 1, &VehicleCmdGate::internalControlCmdCallback, this);
    this->sub_external_control_cmd = nh.subscribe("external_control_cmd", 1, &VehicleCmdGate::externalControlCmdCallback, this);
    this->pub_control_cmd = nh.advertise<ferrari_common::control_cmd>("control_cmd", 1);
    this->srv_set_gate_mode = nh.advertiseService("set_gate_mode", &VehicleCmdGate::setGateModeCallback, this);
    this->srv_is_engage = nh.advertiseService("is_engage", &VehicleCmdGate::isEngageCallback, this);
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
    if (!this->is_engaged)
    {
        ROS_WARN("Vehicle is not engaged, control command is not sent");
        return;
    }

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

bool VehicleCmdGate::isEngageCallback(ferrari_common::is_engage::Request &req,
                                      ferrari_common::is_engage::Response &res)
{
    this->is_engaged = req.is_engage;
    res.success = true;
    return true;
}