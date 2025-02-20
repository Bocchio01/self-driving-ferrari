#pragma once

#include <stdint.h>

class Actuator
{
protected:
    uint16_t valueCurrent;
    uint16_t valueTarget;
    uint16_t valueMin;
    uint16_t valueMax;
    const uint16_t valueReset;
    bool isArmed = false;
    bool isActive = false;

public:
    Actuator(uint16_t valueReset);
    ~Actuator();

    uint16_t setValueTarget(uint16_t valueTarget);
    uint16_t setValueMin(uint16_t valueMin);

    uint16_t setValueMax(uint16_t valueMax);
    uint16_t getValueTarget();
    uint16_t getValueMin();
    uint16_t getValueMax();
    
    bool stop();
    bool start();

    static uint16_t valueRelativeToAbsolute(uint16_t valueRelative, uint16_t valueMin, uint16_t valueMax);
    static uint16_t valueAbsoluteToRelative(uint16_t valueAbsolute, uint16_t valueMin, uint16_t valueMax);

    virtual bool arm() = 0;
    virtual bool disarm() = 0;

    virtual bool update() = 0;
    virtual bool reset() = 0;

    // virtual std::string printInfo() = 0;
};
