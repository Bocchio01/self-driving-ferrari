#include <Arduino.h>
#include <Ramp.h>
#include <rcl/rcl.h>

#include "vehicle/actuators/propulsion.hpp"

ActuatorPropulsion::ActuatorPropulsion(uint8_t PWMA,
                                       uint8_t AIN1,
                                       uint8_t AIN2,
                                       uint8_t STBY,
                                       propulsion_throttle_t throttle_reset,
                                       propulsion_throttle_t throttle_minimum,
                                       propulsion_throttle_t throttle_maximum)
    : PWMA(PWMA),
      AIN1(AIN1),
      AIN2(AIN2),
      STBY(STBY),
      throttle(throttle_reset),
      acceleration_mode(AccelerationMode::BALANCED),
      throttle_reset(throttle_reset),
      throttle_minimum(throttle_minimum),
      throttle_maximum(throttle_maximum),
      rates{50, 100, 250}
{
    pinMode(this->PWMA, OUTPUT);
    pinMode(this->AIN1, OUTPUT);
    pinMode(this->AIN2, OUTPUT);
    pinMode(this->STBY, OUTPUT);
}

ActuatorPropulsion::~ActuatorPropulsion()
{
}

void ActuatorPropulsion::setThrottle(propulsion_throttle_t throttle)
{
    propulsion_throttle_t target = constrain(throttle, this->throttle_minimum, this->throttle_maximum);
    propulsion_throttle_t current = this->throttle.getValue();

    propulsion_throttle_t target_rel = static_cast<propulsion_throttle_t>(target) - static_cast<propulsion_throttle_t>(this->throttle_reset);
    propulsion_throttle_t current_rel = static_cast<propulsion_throttle_t>(current) - static_cast<propulsion_throttle_t>(this->throttle_reset);

    bool is_flipping_direction = (target_rel * current_rel) < 0;
    bool is_accelerating = abs(target_rel) > abs(current_rel);

    unsigned int duration = is_flipping_direction || is_accelerating
                                ? static_cast<int>(this->rates[static_cast<int>(this->acceleration_mode)])
                                : 0;

    this->throttle.go(target, duration, ramp_mode::SINUSOIDAL_INOUT);

    RCUTILS_LOG_DEBUG("ActuatorPropulsion::setThrottle: is_flipping_direction=%d, is_accelerating=%d, duration=%d, target=%d, current=%d",
                      is_flipping_direction,
                      is_accelerating,
                      duration,
                      target,
                      current);
}

bool ActuatorPropulsion::arm()
{
    digitalWrite(this->STBY, HIGH);
    this->is_armed = true;

    return true;
}

bool ActuatorPropulsion::disarm()
{
    digitalWrite(this->STBY, LOW);
    this->is_armed = false;

    return true;
}

bool ActuatorPropulsion::update()
{
    propulsion_throttle_t throttle_current = this->throttle.update();

    if (throttle_current == 0)
    {
        digitalWrite(this->AIN1, LOW);
        digitalWrite(this->AIN2, LOW);
    }
    else
    {
        digitalWrite(this->AIN1, throttle_current > 0 ? HIGH : LOW);
        digitalWrite(this->AIN2, throttle_current < 0 ? HIGH : LOW);
    }

    analogWrite(this->PWMA, constrain(abs(throttle_current), 0, 255));

    RCUTILS_LOG_DEBUG("ActuatorPropulsion::update: throttle_current=%d", throttle_current);

    return true;
}

bool ActuatorPropulsion::reset()
{
    this->throttle.go(this->throttle_reset);
    this->update();

    return true;
}

bool ActuatorPropulsion::brake()
{
    digitalWrite(this->AIN1, HIGH);
    digitalWrite(this->AIN2, HIGH);

    return true;
}