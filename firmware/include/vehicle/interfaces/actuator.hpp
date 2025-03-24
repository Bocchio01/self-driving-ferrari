#pragma once

#include <Arduino.h>

class IActuator
{
protected:
    bool is_armed = false;

public:
    virtual ~IActuator() = default;

    bool isArmed() const { return this->is_armed; }

    virtual bool arm() = 0;
    virtual bool disarm() = 0;
    virtual bool update() = 0;
    virtual bool reset() = 0;

    // template <typename T>
    // static constexpr T map(T x, T in_min, T in_max, T out_min, T out_max)
    // {
    //     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    // }
};
