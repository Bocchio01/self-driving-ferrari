#include "ackermann_adapter.hpp"

AckermannAdapter::AckermannAdapter(ros::NodeHandle &nh) : nh(nh)
{
    this->sub_control_cmd = nh.subscribe("control_cmd", 1, &AckermannAdapter::translateActuationCommand, this);
    this->pub_actuation_cmd = nh.advertise<ferrari_common::ackermann_cmd>("ackermann_actuation_cmd", 1);
}

void AckermannAdapter::translateActuationCommand(const ferrari_common::control_cmd::ConstPtr &control_cmd)
{
    ferrari_common::ackermann_cmd actuation_cmd;

    actuation_cmd.steering_angle = control_cmd->vehicle_yaw_rate;
    actuation_cmd.throttle = control_cmd->vehicle_longitudinal_rate;

    this->pub_actuation_cmd.publish(actuation_cmd);
}
