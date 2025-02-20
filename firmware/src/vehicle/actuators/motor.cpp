#include "vehicle/actuators/motor.hpp"

Motor::Motor(byte pin, byte pinDirection1, byte pinDirection2, uint16_t valueReset)
    : ArduinoComponent(pin),
      Actuator(valueReset),
      pinDirection1(pinDirection1),
      pinDirection2(pinDirection2)
{
    this->valueMin = -255;
    this->valueMax = +255;
}

bool Motor::arm()
{
    this->isArmed = true;

    pinMode(this->pin, OUTPUT);
    pinMode(this->pinDirection1, OUTPUT);
    pinMode(this->pinDirection2, OUTPUT);

    return true;
}

bool Motor::disarm()
{
    this->isArmed = false;

    pinMode(this->pin, INPUT);
    pinMode(this->pinDirection1, INPUT);
    pinMode(this->pinDirection2, INPUT);

    return true;
}

bool Motor::update()
{
    digitalWrite(this->pinDirection1, this->valueTarget > 0 ? HIGH : LOW);
    digitalWrite(this->pinDirection2, this->valueTarget < 0 ? HIGH : LOW);
    analogWrite(this->pin, abs(this->valueTarget));

    this->valueCurrent = this->valueTarget;

    return true;
}

bool Motor::reset()
{
    this->setValueTarget(this->valueReset);
    this->update();

    return true;
}

// std::string Motor::printInfo()
// {
//     return "Motor: " + std::to_string(this->pin) + "\n" +
//            "\tCurrent value: " + std::to_string(this->valueCurrent) + "\n" +
//            "\tTarget value: " + std::to_string(this->valueTarget);
// }
