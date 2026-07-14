#include <Ramp.h>

#include "vehicle/configs.hpp"
#include "vehicle/kinematics/ackermann.hpp"
#include "vehicle/actuators/steering.hpp"
#include "vehicle/actuators/propulsion.hpp"

using namespace vehicle::kinematics;

Ackermann::Ackermann()
    : actuator_steering(nullptr),
      actuator_propulsion(nullptr)
{
    this->kinematic_type = Type::ACKERMANN;
}

void Ackermann::bindActuators(actuators::Steering &actuator_steering, actuators::Propulsion &actuator_propulsion)
{
    this->bindActuatorSteering(actuator_steering);
    this->bindActuatorPropulsion(actuator_propulsion);
}

void Ackermann::bindActuatorSteering(actuators::Steering &actuator_steering)
{
    this->actuator_steering = &actuator_steering;
    this->actuator_steering->reset();
    this->actuator_steering->disarm();
}

void Ackermann::bindActuatorPropulsion(actuators::Propulsion &actuator_propulsion)
{
    this->actuator_propulsion = &actuator_propulsion;
    this->actuator_propulsion->reset();
    this->actuator_propulsion->disarm();
}

void Ackermann::setMotionCommand(const void *motion_cmd_)
{
    using Kinematic = vehicle::configs::Kinematic;

    const ackermann_msgs__msg__AckermannDriveStamped &motion_cmd = *static_cast<const ackermann_msgs__msg__AckermannDriveStamped *>(motion_cmd_);

    float target_steering = constrain(motion_cmd.drive.steering_angle, -Kinematic::MAX_STEERING_ANGLE, Kinematic::MAX_STEERING_ANGLE);
    float target_speed = constrain(motion_cmd.drive.speed, -Kinematic::MAX_SPEED, Kinematic::MAX_SPEED);

    float speed_delta = target_speed - this->actuator_propulsion->getCurrentAngularVelocity() * Kinematic::WHEEL_RADIUS;
    float acceleration = (motion_cmd.drive.acceleration > 0.001f) ? motion_cmd.drive.acceleration : Kinematic::MAX_ACCELERATIONS[static_cast<uint8_t>(this->driving_style)];
    unsigned int speed_duration = abs(target_speed) < 0.15f ? 0 : static_cast<unsigned int>((abs(speed_delta) / acceleration) * 1000.0f);

    float steering_delta = target_steering - this->actuator_steering->getCurrentSteeringAngle();
    unsigned int steering_duration = (motion_cmd.drive.steering_angle_velocity > 0.001f) ? static_cast<unsigned int>((abs(steering_delta) / motion_cmd.drive.steering_angle_velocity) * 1000.0f) : 0;

    this->ramp_target_steering.go(target_steering, steering_duration, ramp_mode::SINUSOIDAL_INOUT);
    this->ramp_target_velocity.go(target_speed, speed_duration, ramp_mode::SINUSOIDAL_INOUT);
}

void Ackermann::update()
{
    using Kinematic = vehicle::configs::Kinematic;

    // Updating the instantaneous target values for the actuators based on the ramped values
    float instantaneous_steering = this->ramp_target_steering.update();
    float instantaneous_speed = this->ramp_target_velocity.update();

    this->actuator_steering->setTargetSteeringAngle(instantaneous_steering);
    this->actuator_propulsion->setTargetAngularVelocity(instantaneous_speed / Kinematic::WHEEL_RADIUS);

    // Updating/moving the actuators to the target values
    this->actuator_steering->update();
    this->actuator_propulsion->update();

    // Updating the odometry state
    unsigned long current_time = micros();
    static unsigned long last_update_time_ = current_time;
    float dt = (current_time - last_update_time_) / 1000000.0f;
    last_update_time_ = current_time;

    // Prevent massive jumps on the first loop or after pauses
    if (dt > 0.1f)
        dt = 0.0f;

    float actual_speed = this->actuator_propulsion->getCurrentAngularVelocity() * Kinematic::WHEEL_RADIUS;
    float actual_steering = this->actuator_steering->getCurrentSteeringAngle();

    this->odometry_state.linear_x = actual_speed;
    this->odometry_state.angular_z = actual_speed * tan(actual_steering) / Kinematic::WHEELBASE;
    this->odometry_state.theta += this->odometry_state.angular_z * dt;
    this->odometry_state.x += actual_speed * cos(this->odometry_state.theta) * dt;
    this->odometry_state.y += actual_speed * sin(this->odometry_state.theta) * dt;
}

void Ackermann::executeEmergencyStop()
{
    this->actuator_steering->reset();
    this->actuator_propulsion->reset();
    this->actuator_propulsion->brake();
}

bool Ackermann::arm()
{
    this->actuator_steering->arm();
    this->actuator_propulsion->arm();

    return true;
}

bool Ackermann::disarm()
{
    this->actuator_steering->disarm();
    this->actuator_propulsion->disarm();

    return true;
}

bool Ackermann::isArmed()
{
    return this->actuator_steering->isArmed() && this->actuator_propulsion->isArmed();
}