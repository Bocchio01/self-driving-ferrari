#pragma once

#include <rcl/rcl.h>
#include "vehicle/interfaces/kinematic.hpp"

class IVehicle
{
public:
    virtual ~IVehicle() = default;

    virtual bool executeArming() = 0;
    virtual bool executeDisarming() = 0;
    virtual void executeMotionCommand(const void *motion_cmd) = 0;
    virtual void executeEmercencyStop() = 0;
    virtual bool isArmed() = 0;
};

template <class TKinematic = IKinematic>
class Vehicle : public IVehicle, public TKinematic
{
public:
    virtual ~Vehicle() = default;

    bool executeArming() override
    {
        return TKinematic::executeArming();
    }

    bool executeDisarming() override
    {
        return TKinematic::executeDisarming();
    }

    void executeMotionCommand(const void *motion_cmd) override
    {
        TKinematic::executeMotionCommand(motion_cmd);
    }

    void executeEmercencyStop() override
    {
        TKinematic::executeEmercencyStop();
    }

    bool isArmed() override
    {
        return TKinematic::isArmed();
    }
};