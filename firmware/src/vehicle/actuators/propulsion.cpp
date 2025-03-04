#include <Arduino.h>
#include "vehicle/actuators/propulsion.hpp"

ActuatorPropulsion::ActuatorPropulsion(uint8_t PWMA,
                                       uint8_t AIN1,
                                       uint8_t AIN2,
                                       uint8_t STBY,
                                       propulsion_throttle_t throttle_reset,
                                       propulsion_throttle_t throttle_minimum,
                                       propulsion_throttle_t throttle_maximum,
                                       Direction direction_reset)
    : PWMA(PWMA),
      AIN1(AIN1),
      AIN2(AIN2),
      STBY(STBY),
      throttle(throttle_reset),
      throttle_current(throttle_reset),
      throttle_reset(throttle_reset),
      throttle_minimum(throttle_minimum),
      throttle_maximum(throttle_maximum),
      direction(Direction::FORWARD),
      direction_reset(direction_reset)
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
    this->throttle = min(this->throttle_maximum, max(this->throttle_minimum, throttle));
}

void ActuatorPropulsion::setDirection(Direction direction)
{
    this->direction = direction;
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
    digitalWrite(this->AIN1, this->direction == Direction::FORWARD ? HIGH : LOW);
    digitalWrite(this->AIN2, this->direction == Direction::BACKWARD ? HIGH : LOW);
    analogWrite(this->PWMA, this->throttle);

    this->throttle_current = this->throttle;

    return true;
}

bool ActuatorPropulsion::reset()
{
    this->setThrottle(this->throttle_reset);
    this->setDirection(this->direction_reset);
    this->update();

    return true;
}
