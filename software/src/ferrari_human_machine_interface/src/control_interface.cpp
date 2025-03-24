#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "ferrari_common/control_cmd.h"
#include "ferrari_common/get_gate_mode.h"
#include "ferrari_common/set_gate_mode.h"
#include "ferrari_common/toggle_arm_vehicle.h"
#include "control_interface.hpp"

ControlInterface::ControlInterface()
{
    this->pub_external_control_cmd = nh.advertise<ferrari_common::control_cmd>("external_control_cmd", 1);
    this->srv_get_gate_mode = nh.serviceClient<ferrari_common::get_gate_mode>("get_gate_mode");
    this->srv_set_gate_mode = nh.serviceClient<ferrari_common::set_gate_mode>("set_gate_mode");
    this->srv_toggle_arm_vehicle = nh.serviceClient<ferrari_common::toggle_arm_vehicle>("toggle_arm_vehicle");
}

ControlInterface::~ControlInterface()
{
}

void ControlInterface::handleSetGateMode()
{
    ferrari_common::get_gate_mode get_gate_mode;
    ferrari_common::set_gate_mode set_gate_mode;

    srv_get_gate_mode.call(get_gate_mode);
    set_gate_mode.request.gate_mode = (get_gate_mode.response.gate_mode + 1) % 2;
    srv_set_gate_mode.call(set_gate_mode);

    if (!set_gate_mode.response.success)
    {
        ROS_ERROR("Failed to change the gate mode");
    }
    else
    {
        ROS_INFO("Gate mode changed to %s", set_gate_mode.request.gate_mode == 0 ? "internal" : "external");
    }
}

void ControlInterface::handleToggleArmVehicle()
{
    ferrari_common::toggle_arm_vehicle toggle_arm_vehicle;
    srv_toggle_arm_vehicle.call(toggle_arm_vehicle);

    if (!toggle_arm_vehicle.response.success)
    {
        ROS_ERROR("Failed to arm/disarm the vehicle");
    }
    else
    {
        ROS_INFO("Vehicle %s", toggle_arm_vehicle.response.is_armed ? "armed" : "disarmed");
    }
}

void ControlInterface::publishControlCmd(int16_t vehicle_yaw_rate, int16_t vehicle_longitudinal_rate, int16_t vehicle_lateral_rate, bool horn)
{
    ferrari_common::control_cmd control_cmd;

    control_cmd.motion_cmd.vehicle_yaw_rate = vehicle_yaw_rate;
    control_cmd.motion_cmd.vehicle_longitudinal_rate = vehicle_longitudinal_rate;
    control_cmd.motion_cmd.vehicle_lateral_rate = vehicle_lateral_rate;
    control_cmd.horn = horn;

    pub_external_control_cmd.publish(control_cmd);
}
