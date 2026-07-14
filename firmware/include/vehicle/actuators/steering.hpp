#pragma once

#include <Servo.h>

#include "vehicle/actuators/actuator.hpp"

namespace vehicle::kinematics
{
    class Ackermann;
    class Differential;
}

namespace vehicle::actuators
{
    class Steering : public vehicle::interfaces::Actuator, public Servo
    {
    public:
        Steering(uint8_t pin_signal);
        ~Steering();

        void setTargetSteeringAngle(const float steering_angle);
        float getCurrentSteeringAngle() const { return this->current_steering_angle; }

        bool arm() override;
        bool disarm() override;
        bool update() override;
        bool reset() override;

        friend class vehicle::kinematics::Ackermann;
        friend class vehicle::kinematics::Differential;

    private:
        const uint8_t pin_signal;

        uint8_t servo_pwm = 0;
        float current_steering_angle = 0.0f;
        float target_steering_angle = 0.0f;
    };
}