#pragma once

#include "vehicle/interfaces/kinematic.hpp"

class IVehicle : public IKinematic
{

public:
    virtual ~IVehicle() = default;
};