#include "servo.hpp"
#include <Arduino.h>
#include "arduinoComponent.hpp"

Servo::Servo(byte pin, uint16_t valueReset)
    : ArduinoComponent(pin),
      Actuator(valueReset)
{
    this->valueMin = 0;
    this->valueMax = 180;
}

bool Servo::arm()
{
    this->isArmed = true;
    this->arduinoServo.attach(this->pin);

    return true;
}

bool Servo::disarm()
{
    this->isArmed = false;
    this->arduinoServo.detach();

    return true;
}

bool Servo::update()
{
    this->arduinoServo.write(this->valueTarget);
    this->valueCurrent = this->valueTarget;

    return true;
}

bool Servo::reset()
{
    this->setValueTarget(this->valueReset);
    this->update();

    return true;
}

void Servo::turn_left()
{
    this->setValueTarget(this->valueMin);
    this->update();

    return;
}

void Servo::turn_right()
{
    this->setValueTarget(this->valueMax);
    this->update();

    return;
}

// std::string Servo::printInfo()
// {
//     return "Servo: " + std::to_string(this->pin) + "\n" +
//            "\tCurrent angle: " + std::to_string(this->valueCurrent) + "\n" +
//            "\tTarget angle: " + std::to_string(this->valueTarget) + "\n" +
//            "\tMin angle: " + std::to_string(this->valueMin) + "\n" +
//            "\tMax angle: " + std::to_string(this->valueMax);
// }
