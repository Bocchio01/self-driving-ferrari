#pragma once

#include <geometry_msgs/msg/twist.h>
#include "vehicle/kinematics/kinematic.hpp"
#include "vehicle/actuators/actuator.hpp"
#include "vehicle/actuators/propulsion.hpp"

namespace vehicle::kinematics
{
    class Differential : public vehicle::interfaces::Kinematic
    {
    public:
        enum class ActuatorSide
        {
            LEFT,
            RIGHT
        };

        Differential();

        void bindActuatorPropulsion(vehicle::actuators::Propulsion &actuator_propulsion, ActuatorSide side);

        void setMotionCommand(const void *motion_cmd);
        bool arm();
        bool disarm();
        bool isArmed();

    private:
        vehicle::actuators::Propulsion *actuator_propulsion_right;
        vehicle::actuators::Propulsion *actuator_propulsion_left;
    };

}