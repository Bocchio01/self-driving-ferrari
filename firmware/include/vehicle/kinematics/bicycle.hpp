#pragma once

#include "vehicle/virtuals/kinematic.hpp"
#include "vehicle/actuators/servo.hpp"
#include "vehicle/actuators/motor.hpp"

class Bicycle : public Kinematic {
private:
    MyServo& steering;
    Motor& drive;

public:
    Bicycle(MyServo& steering, Motor& drive);
    ~Bicycle();

    void move(double steeringAngle, double velocity) override;
    // static std::string type() { return "Bicycle"; }
};
