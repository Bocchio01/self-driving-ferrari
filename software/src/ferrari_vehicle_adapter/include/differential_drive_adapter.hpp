#pragma once

#include <ros/ros.h>
#include "kinematic_adapter_interface.hpp"
#include "ferrari_common/control_cmd.h"
#include "ferrari_common/differential_drive_cmd.h"

class DifferentialDriveAdapter : public KinematicAdapterInterface
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_control_cmd;
    ros::Publisher pub_actuation_cmd;

public:
    DifferentialDriveAdapter(ros::NodeHandle &nh);
    void translateActuationCommand(const ferrari_common::control_cmd::ConstPtr &control_cmd) override;
};
