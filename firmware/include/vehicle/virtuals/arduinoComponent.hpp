#pragma once

#include <Arduino.h>

class ArduinoComponent
{
protected:
    const byte pin;

public:
    ArduinoComponent(byte pin) : pin(pin) {};
    virtual ~ArduinoComponent() {};
};