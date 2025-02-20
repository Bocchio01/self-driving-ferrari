#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "vehicle/virtuals/actuator.hpp"
#include "vehicle/virtuals/arduinoComponent.hpp"

class Motor : public Actuator, public ArduinoComponent
{
private:
    const byte pinDirection1;
    const byte pinDirection2;

public:
    Motor(byte pin, byte pinDirection1, byte pinDirection2, uint16_t valueReset = 0);

    bool arm() override;
    bool disarm() override;

    bool update() override;
    bool reset() override;

    // std::string printInfo() override;
};
