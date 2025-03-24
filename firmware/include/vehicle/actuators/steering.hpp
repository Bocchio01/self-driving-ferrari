#pragma once

#include <Servo.h>
#include "vehicle/interfaces/actuator.hpp"

class ActuatorSteering : public IActuator, public Servo
{
    typedef int16_t steering_angle_t;

private:
    const uint8_t pin_signal;

private:
    steering_angle_t steering_angle;
    steering_angle_t steering_angle_current;
    const steering_angle_t steering_angle_reset;
    const steering_angle_t steering_angle_minimum;
    const steering_angle_t steering_angle_maximum;

public:
    ActuatorSteering(uint8_t pin_signal,
                     steering_angle_t steering_angle_reset = 90,
                     steering_angle_t steering_angle_minimum = 0,
                     steering_angle_t steering_angle_maximum = 180);
    ~ActuatorSteering();

    bool arm() override;
    bool disarm() override;
    bool update() override;
    bool reset() override;

    void setSteeringAngle(steering_angle_t angle);

    friend class KinematicAckermann;
    friend class KinematicDifferential;
};