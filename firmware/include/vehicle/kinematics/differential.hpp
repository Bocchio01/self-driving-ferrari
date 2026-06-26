#pragma once

#include <geometry_msgs/msg/twist.h>
#include "vehicle/interfaces/kinematic.hpp"
#include "vehicle/actuators/propulsion.hpp"

class KinematicDifferential : public IKinematic
{
public:
    enum class ActuatorPropulsionSide
    {
        LEFT,
        RIGHT
    };

private:
    ActuatorPropulsion *actuator_propulsion_right;
    ActuatorPropulsion *actuator_propulsion_left;

public:
    KinematicDifferential();

    void bindActuatorPropulsion(ActuatorPropulsion &actuator_propulsion, ActuatorPropulsionSide side);

    void executeMotionCommand(const void *motion_cmd);
    bool executeArming();
    bool executeDisarming();
    bool isArmed();
};
