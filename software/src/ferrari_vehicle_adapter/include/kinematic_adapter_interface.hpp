#pragma once

#include <ros/ros.h>
#include "ferrari_common/control_cmd.h"

enum class KinematicModel
{
    ACKERMANN,
    DIFFERENTIAL_DRIVE,
    OMNIDIRECTIONAL
};

class KinematicAdapterInterface
{
public:
    virtual void translateActuationCommand(const ferrari_common::control_cmd::ConstPtr &cmd) = 0;

public:
    virtual ~KinematicAdapterInterface() = default;
};
