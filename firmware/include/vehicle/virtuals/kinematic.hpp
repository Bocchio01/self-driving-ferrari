#pragma once

class Kinematic {
public:
    Kinematic() {}
    virtual ~Kinematic() {}

    virtual void move(double input1, double input2) = 0;
};
