#pragma once

#include "ferrari_common/control_cmd.h"
#include "vehicle/vehicle_interface.hpp"

template <class TKinematic>
class Vehicle : public TKinematic, public IVehicle
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

    void executeMotionCommand(const ferrari_common::motion_cmd &motion_cmd) override
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