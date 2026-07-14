#pragma once

#include <Arduino.h>

namespace vehicle::interfaces
{
    class Actuator
    {
    protected:
        bool is_armed = false;

    public:
        virtual ~Actuator() = default;

        bool isArmed() const { return this->is_armed; }

        virtual bool arm() = 0;
        virtual bool disarm() = 0;
        virtual bool update() = 0;
        virtual bool reset() = 0;
    };
}