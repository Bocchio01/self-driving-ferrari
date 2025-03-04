#include "vehicle/interfaces/kinematic.hpp"
#include "vehicle/kinematics/ackermann.hpp"
#include "vehicle/actuators/steering.hpp"
#include "vehicle/actuators/propulsion.hpp"

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
    using Direction = ActuatorPropulsion::Direction;

    uint8_t steering_angle = IActuator::map<float>(motion_cmd.vehicle_yaw_rate, -1, 1, this->actuator_steering->steering_angle_minimum, this->actuator_steering->steering_angle_maximum);
    uint8_t propulsion_throttle = IActuator::map<float>(abs(motion_cmd.vehicle_longitudinal_rate), 0, 1, this->actuator_propulsion->throttle_minimum, this->actuator_propulsion->throttle_maximum);
    Direction propulsion_direction = motion_cmd.vehicle_longitudinal_rate > 0 ? Direction::FORWARD : Direction::BACKWARD;

    this->actuator_steering->setSteeringAngle(steering_angle);
    this->actuator_propulsion->setThrottle(propulsion_throttle);
    this->actuator_propulsion->setDirection(propulsion_direction);

    this->actuator_steering->update();
    this->actuator_propulsion->update();
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