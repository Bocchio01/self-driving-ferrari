#pragma once

#include <Arduino.h>
#include "vehicle/interfaces/actuator.hpp"

typedef float propulsion_throttle_t;

class ActuatorPropulsion : public IActuator
{

public:
    enum class Direction
    {
        FORWARD,
        BACKWARD
    };

private:
    const uint8_t PWMA;
    const uint8_t AIN1;
    const uint8_t AIN2;
    const uint8_t STBY;

private:
    propulsion_throttle_t throttle;
    propulsion_throttle_t throttle_current;
    const propulsion_throttle_t throttle_reset;
    const propulsion_throttle_t throttle_minimum;
    const propulsion_throttle_t throttle_maximum;

    Direction direction;
    const Direction direction_reset;

public:
    ActuatorPropulsion(uint8_t PWMA,
                       uint8_t AIN1,
                       uint8_t AIN2,
                       uint8_t STBY,
                       propulsion_throttle_t throttle_reset = 0,
                       propulsion_throttle_t throttle_minimum = 0,
                       propulsion_throttle_t throttle_maximum = 255,
                       Direction direction_reset = Direction::FORWARD);
    ~ActuatorPropulsion();

    bool arm() override;
    bool disarm() override;
    bool update() override;
    bool reset() override;

    void setThrottle(propulsion_throttle_t throttle);
    void setDirection(Direction direction);

    friend class KinematicAckermann;
    friend class KinematicDifferential;
};