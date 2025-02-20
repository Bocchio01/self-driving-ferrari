#pragma once

#include <stdint.h>
#include <Servo.h>

#include "vehicle/virtuals/actuator.hpp"
#include "vehicle/virtuals/arduinoComponent.hpp"

class Servo : public Actuator, public ArduinoComponent
{
private:
    ::Servo arduinoServo;

public:
    Servo(byte pin, uint16_t valueReset = 90);

    bool arm() override;
    bool disarm() override;

    bool update() override;
    bool reset() override;

    // std::string printInfo() override;

    void turn_left();
    void turn_right();
};
