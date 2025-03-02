#include "differential_drive_adapter.hpp"

DifferentialDriveAdapter::DifferentialDriveAdapter(ros::NodeHandle &nh) : nh(nh)
{
    this->sub_control_cmd = nh.subscribe("control_cmd", 1, &DifferentialDriveAdapter::translateActuationCommand, this);
    this->pub_actuation_cmd = nh.advertise<ferrari_common::differential_drive_cmd>("differential_drive_actuation_cmd", 1);
}

void DifferentialDriveAdapter::translateActuationCommand(const ferrari_common::control_cmd::ConstPtr &control_cmd)
{
    ferrari_common::differential_drive_cmd actuation_cmd;

    actuation_cmd.throttle_left = control_cmd->vehicle_yaw_rate - control_cmd->vehicle_longitudinal_rate / 2;
    actuation_cmd.throttle_right = control_cmd->vehicle_yaw_rate + control_cmd->vehicle_longitudinal_rate / 2;

    this->pub_actuation_cmd.publish(actuation_cmd);
}
