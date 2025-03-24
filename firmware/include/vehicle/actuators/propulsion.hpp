#pragma once

#include <Arduino.h>
#include <Ramp.h>
#include "vehicle/interfaces/actuator.hpp"

class ActuatorPropulsion : public IActuator
{
    typedef int16_t propulsion_throttle_t;

public:
    enum class AccelerationMode
    {
        CALM,
        BALANCED,
        NERVOUS
    };

private:
    const uint8_t PWMA;
    const uint8_t AIN1;
    const uint8_t AIN2;
    const uint8_t STBY;
    _ramp<propulsion_throttle_t> throttle;
    AccelerationMode acceleration_mode;

private:
    const propulsion_throttle_t throttle_reset;
    const propulsion_throttle_t throttle_minimum;
    const propulsion_throttle_t throttle_maximum;
    const uint8_t rates[3];

public:
    ActuatorPropulsion(uint8_t PWMA,
                       uint8_t AIN1,
                       uint8_t AIN2,
                       uint8_t STBY,
                       propulsion_throttle_t throttle_reset = +0,
                       propulsion_throttle_t throttle_minimum = -255,
                       propulsion_throttle_t throttle_maximum = +255);
    ~ActuatorPropulsion();

    bool arm() override;
    bool disarm() override;
    bool update() override;
    bool reset() override;
    bool brake();

    void setThrottle(propulsion_throttle_t throttle);

    friend class KinematicAckermann;
    friend class KinematicDifferential;
    friend class PublisherHelloWorld;
};