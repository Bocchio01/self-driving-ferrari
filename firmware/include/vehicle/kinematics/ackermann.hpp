#pragma once

#include <Ramp.h>
#include <ackermann_msgs/msg/ackermann_drive_stamped.h>

#include "vehicle/kinematics/kinematic.hpp"
#include "vehicle/actuators/steering.hpp"
#include "vehicle/actuators/propulsion.hpp"

namespace vehicle::kinematics
{
    class Ackermann : public vehicle::interfaces::Kinematic
    {

    public:
        enum class DrivingStyle : uint8_t
        {
            CALM,
            BALANCED,
            NERVOUS
        };

        Ackermann();

        void bindActuators(vehicle::actuators::Steering &actuator_steering, vehicle::actuators::Propulsion &actuator_propulsion);
        void bindActuatorSteering(vehicle::actuators::Steering &actuator_steering);
        void bindActuatorPropulsion(vehicle::actuators::Propulsion &actuator_propulsion);

        void setMotionCommand(const void *motion_cmd);
        void update();
        void executeEmergencyStop();
        bool arm();
        bool disarm();
        bool isArmed();

    private:
        DrivingStyle driving_style = DrivingStyle::BALANCED;

        vehicle::actuators::Steering *actuator_steering;
        vehicle::actuators::Propulsion *actuator_propulsion;

        rampFloat ramp_target_steering;
        rampFloat ramp_target_velocity;
    };
}