#include <Arduino.h>
#include <pid.hpp>

#include "vehicle/configs.hpp"
#include "vehicle/actuators/propulsion.hpp"

using namespace vehicle::actuators;
using namespace vehicle::configs;

Propulsion::Propulsion(uint8_t PWMA,
                       uint8_t AIN1,
                       uint8_t AIN2,
                       uint8_t STBY)
    : PWMA(PWMA),
      AIN1(AIN1),
      AIN2(AIN2),
      STBY(STBY)
{
    pinMode(this->PWMA, OUTPUT);
    pinMode(this->AIN1, OUTPUT);
    pinMode(this->AIN2, OUTPUT);
    pinMode(this->STBY, OUTPUT);

    this->setTargetAngularPosition(0.0f);
    this->setTargetAngularVelocity(0.0f);
}

Propulsion::~Propulsion()
{
}

void Propulsion::setControllerType(ControllerType controller_type)
{
    if (this->controller_type == controller_type)
        return;

    this->controller_type = controller_type;

    if (this->controller_type == ControllerType::SPEED && this->controller_angular_velocity != nullptr)
    {
        this->controller_angular_velocity->reset();
    }
    else if (this->controller_type == ControllerType::POSITION && this->controller_angular_position != nullptr)
    {
        this->controller_angular_position->reset();
    }
}

void Propulsion::setTargetAngularVelocity(float target_angular_velocity)
{
    this->setControllerType(ControllerType::SPEED);
    this->target_angular_velocity = target_angular_velocity;
    this->target_angular_position = NAN;
    this->target_angular_position_last = NAN;
}

void Propulsion::setTargetAngularPosition(float target_angular_position)
{
    this->setControllerType(ControllerType::POSITION);
    this->target_angular_velocity = NAN;
    this->target_angular_position = target_angular_position;
    if (isnan(this->target_angular_position_last))
    {
        this->target_angular_position_last = target_angular_position;
    }
}

void Propulsion::setControllerAngularVelocity(PID *controller_angular_velocity, FeedbackCallback feedback_callback)
{
    if (controller_angular_velocity == nullptr || feedback_callback == nullptr)
    {
        return;
    }

    this->controller_angular_velocity = controller_angular_velocity;
    this->feedback_callback_angular_velocity = feedback_callback;
    this->controller_angular_velocity->setOutputLimits(Actuators::MIN_MOTOR_PWM, Actuators::MAX_MOTOR_PWM);
}

void Propulsion::setControllerAngularPosition(PID *controller_angular_position, FeedbackCallback feedback_callback)
{
    if (controller_angular_position == nullptr || feedback_callback == nullptr)
    {
        return;
    }

    this->controller_angular_position = controller_angular_position;
    this->feedback_callback_angular_position = feedback_callback;
    this->controller_angular_position->setOutputLimits(Actuators::MIN_MOTOR_PWM, Actuators::MAX_MOTOR_PWM);
}

bool Propulsion::arm()
{
    if (this->is_armed)
    {
        return true;
    }

    this->reset();
    digitalWrite(this->STBY, HIGH);

    this->is_armed = true;

    return true;
}

bool Propulsion::disarm()
{
    if (!this->is_armed)
    {
        return true;
    }

    this->reset();
    digitalWrite(this->STBY, LOW);

    this->is_armed = false;

    return true;
}

bool Propulsion::update()
{
    this->current_angular_velocity = this->feedback_callback_angular_velocity != nullptr ? this->feedback_callback_angular_velocity() : 0.0f;
    this->current_angular_position = this->feedback_callback_angular_position != nullptr ? this->feedback_callback_angular_position() : 0.0f;

    if (!this->is_armed)
    {
        return false;
    }

    float motor_pwm = 0.0f;

    switch (this->controller_type)
    {

    // We apply a feedforward term based on the target speed + a feedback term based on the speed error.
    case ControllerType::SPEED:
    {
        motor_pwm += Actuators::ANGULAR_VELOCITY_TO_MOTOR_PWM(this->target_angular_velocity);
        if (this->feedback_callback_angular_velocity != nullptr && this->controller_angular_velocity != nullptr)
        {
            motor_pwm += this->controller_angular_velocity->compute(this->target_angular_velocity, this->current_angular_velocity);
        }

        break;
    }

    // We apply a feedforward term based on ideal velocity + a feedback term based on the position error.
    // The ideal velocity is computed as temporal derivative of the ramped target position.
    case ControllerType::POSITION:
    {
        float feedforward_velocity = (this->target_angular_position - this->target_angular_position_last) * Actuators::LOOP_RATE;
        this->target_angular_position_last = this->target_angular_position;

        motor_pwm += Actuators::ANGULAR_VELOCITY_TO_MOTOR_PWM(feedforward_velocity);
        if (this->feedback_callback_angular_position != nullptr && this->controller_angular_position != nullptr)
        {
            motor_pwm += this->controller_angular_position->compute(this->target_angular_position, this->current_angular_position);
        }

        break;
    }
    }

    // To avoid motor noise and unnecessary power consumption, we can set a deadband around zero PWM.
    this->motor_pwm = abs(motor_pwm) > 3 ? constrain(static_cast<int16_t>(motor_pwm), Actuators::MIN_MOTOR_PWM, Actuators::MAX_MOTOR_PWM) : 0;

    digitalWrite(this->AIN1, this->motor_pwm > 0 ? HIGH : LOW);
    digitalWrite(this->AIN2, this->motor_pwm < 0 ? HIGH : LOW);
    analogWrite(this->PWMA, constrain(abs(this->motor_pwm), 0, 255));

    return true;
}

bool Propulsion::reset()
{
    this->target_angular_velocity = NAN;
    this->target_angular_position = NAN;
    this->target_angular_position_last = NAN;

    if (this->controller_angular_velocity != nullptr)
    {
        this->controller_angular_velocity->reset();
    }

    if (this->controller_angular_position != nullptr)
    {
        this->controller_angular_position->reset();
    }

    this->motor_pwm = 0;
    digitalWrite(this->AIN1, LOW);
    digitalWrite(this->AIN2, LOW);
    analogWrite(this->PWMA, 0);

    return true;
}

bool Propulsion::brake()
{
    this->reset();
    digitalWrite(this->AIN1, HIGH);
    digitalWrite(this->AIN2, HIGH);

    return true;
}