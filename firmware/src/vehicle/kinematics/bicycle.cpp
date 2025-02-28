#include "vehicle/virtuals/actuator.hpp"
#include "vehicle/kinematics/bicycle.hpp"
#include "vehicle/actuators/servo.hpp"
#include "vehicle/actuators/motor.hpp"

Bicycle::Bicycle(MyServo &steering, Motor &drive)
    : steering(steering),
      drive(drive)
{
}

Bicycle::~Bicycle()
{
}

void Bicycle::move(double steeringAngle, double velocity)
{
    steering.setValueTarget(Actuator::valueRelativeToAbsolute(steeringAngle, 0, 180));
    drive.setValueTarget(Actuator::valueRelativeToAbsolute(velocity, -255, 255));
    
    steering.update();
    drive.update();
}