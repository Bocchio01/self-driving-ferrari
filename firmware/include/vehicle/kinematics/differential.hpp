#pragma once

#include "vehicle/virtuals/kinematic.hpp"
#include "vehicle/actuators/motor.hpp"

class Differential : public Kinematic {
private:
    Motor& leftMotor;
    Motor& rightMotor;

public:
    Differential(Motor& leftMotor, Motor& rightMotor);
    ~Differential();

    void move(double leftSpeed, double rightSpeed) override;
    // static std::string type() { return "Differential"; }
};
