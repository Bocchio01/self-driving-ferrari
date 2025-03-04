#include <Arduino.h>
#include <Servo.h>
#include "vehicle/actuators/steering.hpp"

ActuatorSteering::ActuatorSteering(uint8_t pin_signal,
                                   steering_angle_t steering_angle_reset,
                                   steering_angle_t steering_angle_minimum,
                                   steering_angle_t steering_angle_maximum)
    : IActuator(),
      Servo(),
      pin_signal(pin_signal),
      steering_angle(steering_angle_reset),
      steering_angle_current(steering_angle_reset),
      steering_angle_reset(steering_angle_reset),
      steering_angle_minimum(steering_angle_minimum),
      steering_angle_maximum(steering_angle_maximum)
{
}

ActuatorSteering::~ActuatorSteering()
{
}

void ActuatorSteering::setSteeringAngle(steering_angle_t steering_angle)
{
    this->steering_angle = min(this->steering_angle_maximum, max(this->steering_angle_minimum, steering_angle));
}

bool ActuatorSteering::arm()
{
    this->attach(this->pin_signal);
    this->write(this->steering_angle);
    this->is_armed = true;

    return this->attached();
}

bool ActuatorSteering::disarm()
{
    this->detach();
    this->is_armed = false;

    return true;
}

bool ActuatorSteering::update()
{
    if (!this->is_armed)
    {
        return false;
    }

    if (this->steering_angle_current != this->steering_angle)
    {
        this->write(this->steering_angle);
        this->steering_angle_current = this->steering_angle;
    }

    return true;
}

bool ActuatorSteering::reset()
{
    this->setSteeringAngle(this->steering_angle_reset);
    this->write(this->steering_angle);

    return true;
}