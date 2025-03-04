#pragma once

#include "ferrari_common/control_cmd.h"
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

    void executeMotionCommand(const ferrari_common::motion_cmd &motion_cmd);
    bool executeArming();
    bool executeDisarming();
    bool isArmed();
};
