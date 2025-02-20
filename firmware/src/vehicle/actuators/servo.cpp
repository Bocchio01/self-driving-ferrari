#include "vehicle/actuators/servo.hpp"
#include "vehicle/virtuals/actuator.hpp"
#include "vehicle/virtuals/arduinoComponent.hpp"

MyServo::MyServo(byte pin, uint16_t valueReset)
    : Actuator(valueReset),
      ArduinoComponent(pin)
{
    this->valueMin = 0;
    this->valueMax = 180;
}

bool MyServo::arm()
{
    this->isArmed = true;
    this->arduinoServo.attach(this->pin);

    return true;
}

bool MyServo::disarm()
{
    this->isArmed = false;
    this->arduinoServo.detach();

    return true;
}

bool MyServo::update()
{
    this->arduinoServo.write(this->valueTarget);
    this->valueCurrent = this->valueTarget;

    return true;
}

bool MyServo::reset()
{
    this->setValueTarget(this->valueReset);
    this->update();

    return true;
}

void MyServo::turn_left()
{
    this->setValueTarget(this->valueMin);
    this->update();

    return;
}

void MyServo::turn_right()
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
