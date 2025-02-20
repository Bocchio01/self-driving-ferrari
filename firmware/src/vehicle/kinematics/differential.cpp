#include "vehicle/kinematics/differential.hpp"
#include "vehicle/actuators/servo.hpp"
#include "vehicle/actuators/motor.hpp"

Differential::Differential(Motor &leftMotor, Motor &rightMotor)
    : leftMotor(leftMotor),
      rightMotor(rightMotor)
{
}

Differential::~Differential()
{
}

void Differential::move(double leftSpeed, double rightSpeed)
{
    printf("Differential Model\n");
    printf("Left speed\t: %f\n", leftSpeed);
    printf("Right speed\t: %f\n", rightSpeed);
}