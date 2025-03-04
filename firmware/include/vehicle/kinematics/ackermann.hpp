#pragma once

#include "ferrari_common/control_cmd.h"
#include "vehicle/interfaces/kinematic.hpp"
#include "vehicle/actuators/steering.hpp"
#include "vehicle/actuators/propulsion.hpp"

class KinematicAckermann : public IKinematic
{
private:
    ActuatorSteering *actuator_steering;
    ActuatorPropulsion *actuator_propulsion;

public:
    KinematicAckermann();

    void bindActuatorSteering(ActuatorSteering &actuator_steering);
    void bindActuatorPropulsion(ActuatorPropulsion &actuator_propulsion);

    void executeMotionCommand(const ferrari_common::motion_cmd &motion_cmd);
    bool executeArming();
    bool executeDisarming();
    bool isArmed();
};
