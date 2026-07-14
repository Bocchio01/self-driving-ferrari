#pragma once

#include <Arduino.h>
#include <pid.hpp>

#include "vehicle/actuators/actuator.hpp"

namespace vehicle::kinematics
{
    class Ackermann;
    class Differential;
}

namespace vehicle::actuators
{
    class Propulsion : public interfaces::Actuator
    {
    public:
        using FeedbackCallback = std::function<float()>;

        enum class ControllerType : uint8_t
        {
            SPEED,
            POSITION
        };

        Propulsion(uint8_t PWMA,
                   uint8_t AIN1,
                   uint8_t AIN2,
                   uint8_t STBY);
        ~Propulsion();

        void setControllerType(ControllerType controller_type);

        void setTargetAngularVelocity(float target_angular_velocity);
        void setTargetAngularPosition(float target_angular_position);

        void setControllerAngularVelocity(PID *controller_angular_velocity, FeedbackCallback feedback_callback);
        void setControllerAngularPosition(PID *controller_angular_position, FeedbackCallback feedback_callback);

        float getCurrentAngularVelocity() const { return this->current_angular_velocity; }
        float getCurrentAngularPosition() const { return this->current_angular_position; }

        bool arm() override;
        bool disarm() override;
        bool update() override;
        bool reset() override;
        bool brake();

        friend class vehicle::kinematics::Ackermann;
        friend class vehicle::kinematics::Differential;

    private:
        const uint8_t PWMA;
        const uint8_t AIN1;
        const uint8_t AIN2;
        const uint8_t STBY;

        float current_angular_velocity = NAN;
        float current_angular_position = NAN;
        float target_angular_velocity = NAN;
        float target_angular_position = NAN;
        float target_angular_position_last = NAN;

        PID *controller_angular_velocity = nullptr;
        PID *controller_angular_position = nullptr;

        FeedbackCallback feedback_callback_angular_velocity = nullptr;
        FeedbackCallback feedback_callback_angular_position = nullptr;

        ControllerType controller_type = ControllerType::SPEED;
        int16_t motor_pwm = 0; // [unitless] [-255, +255]
    };
}