#pragma once

#include <ackermann_msgs/msg/ackermann_drive_stamped.h>
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

    void executeMotionCommand(const void *motion_cmd);
    void executeEmercencyStop();
    bool executeArming();
    bool executeDisarming();
    bool isArmed();
};
