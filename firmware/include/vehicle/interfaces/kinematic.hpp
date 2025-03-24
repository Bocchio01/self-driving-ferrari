#pragma once

#include "ferrari_common/control_cmd.h"

class IKinematic
{
public:
    enum class Type
    {
        ACKERMANN,
        DIFFERENTIAL,
        OMNIDIRECTIONAL,
        UNKNOWN
    };

protected:
    Type kinematic_type;

public:
    // IKinematic(Type kinematic_type) : kinematic_type(kinematic_type) {}
    virtual ~IKinematic() = default;

    Type getType() const { return this->kinematic_type; }

    virtual void executeMotionCommand(const ferrari_common::motion_cmd &motion_cmd) = 0;
    virtual void executeEmercencyStop() = 0;
    virtual bool executeArming() = 0;
    virtual bool executeDisarming() = 0;
    virtual bool isArmed() = 0;
};