#include "vehicle/interfaces/kinematic.hpp"
#include "vehicle/kinematics/ackermann.hpp"
#include "vehicle/actuators/steering.hpp"
#include "vehicle/actuators/propulsion.hpp"
#include <ros.h>

KinematicAckermann::KinematicAckermann()
    : actuator_steering(nullptr),
      actuator_propulsion(nullptr)
{
    this->kinematic_type = Type::ACKERMANN;
}

void KinematicAckermann::bindActuatorSteering(ActuatorSteering &actuator_steering)
{
    this->actuator_steering = &actuator_steering;
    this->actuator_steering->reset();
    this->actuator_steering->disarm();
}

void KinematicAckermann::bindActuatorPropulsion(ActuatorPropulsion &actuator_propulsion)
{
    this->actuator_propulsion = &actuator_propulsion;
    this->actuator_propulsion->reset();
    this->actuator_propulsion->disarm();
}

void KinematicAckermann::executeMotionCommand(const ferrari_common::motion_cmd &motion_cmd)
{
    using steering_angle_t = ActuatorSteering::steering_angle_t;
    using throttle_t = ActuatorPropulsion::propulsion_throttle_t;

    steering_angle_t steering_angle = map(motion_cmd.vehicle_yaw_rate, -100, +100, this->actuator_steering->steering_angle_minimum, this->actuator_steering->steering_angle_maximum);
    throttle_t propulsion_throttle = map(motion_cmd.vehicle_longitudinal_rate, -100, +100, this->actuator_propulsion->throttle_minimum, this->actuator_propulsion->throttle_maximum);

    this->actuator_steering->setSteeringAngle(steering_angle);
    this->actuator_propulsion->setThrottle(propulsion_throttle);

    // this->actuator_steering->update();
    // this->actuator_propulsion->update();
}

void KinematicAckermann::executeEmercencyStop()
{
    this->actuator_steering->reset();
    this->actuator_propulsion->reset();
    this->actuator_propulsion->brake();
}

bool KinematicAckermann::executeArming()
{
    this->actuator_steering->arm();
    this->actuator_propulsion->arm();

    return true;
}

bool KinematicAckermann::executeDisarming()
{
    this->actuator_steering->disarm();
    this->actuator_propulsion->disarm();

    return true;
}

bool KinematicAckermann::isArmed()
{
    return this->actuator_steering->isArmed() && this->actuator_propulsion->isArmed();
}