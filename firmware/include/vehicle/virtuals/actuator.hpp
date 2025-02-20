#pragma once

// #include <string>
#include <stdint.h>
#include <Arduino.h>

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

Actuator::Actuator(uint16_t valueReset)
    : valueCurrent(valueReset),
      valueTarget(valueReset),
      valueMin(0),
      valueMax(100),
      valueReset(valueReset) {}

Actuator::~Actuator()
{
    this->stop();
}

uint16_t Actuator::setValueTarget(uint16_t valueTarget)
{
    if (valueTarget < this->valueMin)
        valueTarget = this->valueMin;
    if (valueTarget > this->valueMax)
        valueTarget = this->valueMax;

    this->valueTarget = valueTarget;

    return this->getValueTarget();
};

uint16_t Actuator::setValueMin(uint16_t valueMin)
{
    this->valueMin = valueMin;
    return this->getValueMin();
};

uint16_t Actuator::setValueMax(uint16_t valueMax)
{
    this->valueMax = valueMax;
    return this->getValueMax();
};

uint16_t Actuator::getValueTarget()
{
    return this->valueTarget;
};

uint16_t Actuator::getValueMin()
{
    return this->valueMin;
};

uint16_t Actuator::getValueMax()
{
    return this->valueMax;
};

bool Actuator::stop()
{
    return this->disarm();
}

bool Actuator::start()
{
    return this->arm();
}

uint16_t Actuator::valueRelativeToAbsolute(uint16_t valueRelative, uint16_t valueMin, uint16_t valueMax)
{
    return map(valueRelative, -1, 1, valueMin, valueMax);
}

uint16_t Actuator::valueAbsoluteToRelative(uint16_t valueAbsolute, uint16_t valueMin, uint16_t valueMax)
{
    return map(valueAbsolute, valueMin, valueMax, -1, 1);
}