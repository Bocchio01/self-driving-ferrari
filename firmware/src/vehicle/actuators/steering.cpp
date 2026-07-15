#include <Arduino.h>
#include <Servo.h>

#include "vehicle/configs.hpp"
#include "vehicle/actuators/steering.hpp"

using namespace vehicle::actuators;

Steering::Steering(uint8_t pin_signal)
    : pin_signal(pin_signal)
{
    this->setTargetSteeringAngle(0.0f);
}

Steering::~Steering()
{
}

void Steering::setTargetSteeringAngle(const float target_steering_angle)
{
    this->target_steering_angle = target_steering_angle;
}

bool Steering::arm()
{
    this->is_armed = true;
    this->attach(this->pin_signal);
    this->reset();

    return this->attached();
}

bool Steering::disarm()
{
    this->is_armed = false;
    this->detach();

    return true;
}

bool Steering::update()
{
    if (!this->is_armed)
    {
        return false;
    }

    this->current_steering_angle = this->target_steering_angle;
    this->servo_pwm = vehicle::configs::Actuators::STEERING_ANGLE_TO_SERVO_PWM(this->target_steering_angle);
    this->current_microseconds = static_cast<uint16_t>(MIN_PULSE_WIDTH + (this->servo_pwm * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180.0f));
    this->writeMicroseconds(this->current_microseconds);

    return true;
}

bool Steering::reset()
{
    this->setTargetSteeringAngle(0.0f);
    this->update();

    return true;
}