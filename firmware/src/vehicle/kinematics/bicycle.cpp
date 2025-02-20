#include "vehicle/kinematics/bicycle.hpp"
#include "vehicle/actuators/servo.hpp"
#include "vehicle/actuators/motor.hpp"

Bicycle::Bicycle(Servo &steering, Motor &drive)
    : steering(steering),
      drive(drive)
{
}

Bicycle::~Bicycle()
{
}

void Bicycle::move(double steeringAngle, double velocity)
{
    printf("Bicycle Model\n");
    printf("Steering\t: %f\n", steeringAngle);
    printf("Velocity\t: %f\n", velocity);

    // steering.set_angle(steeringAngle);
    // drive.set_speed(velocity);
}