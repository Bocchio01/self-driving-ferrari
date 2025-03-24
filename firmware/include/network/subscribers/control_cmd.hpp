#pragma once

#include <Arduino.h>
#include "network/ros.h"
#include "vehicle/vehicle_interface.hpp"
#include "network/virtuals/subscriber.hpp"
#include "ferrari_common/control_cmd.h"

class SubscriberControlCmd : public Subscriber
{
private:
    ros::Subscriber<ferrari_common::control_cmd, SubscriberControlCmd> sub;
    unsigned long last_cmd_time = 0;
    const unsigned long TIMEOUT_MS = 2000;

    void cmdCallback(const ferrari_common::control_cmd &control_cmd)
    {
        if (this->vehicle == nullptr)
        {
            return;
        }

        this->last_cmd_time = millis();
        this->vehicle->executeMotionCommand(control_cmd.motion_cmd);
        // this->vehicle->accessories->setHornCommand(control_cmd.horn);
    }

public:
    SubscriberControlCmd()
        : sub("control_cmd", &SubscriberControlCmd::cmdCallback, this) {}

    void init(ros::NodeHandle &nh, IVehicle &vehicle) override
    {
        this->vehicle = &vehicle;
        this->last_cmd_time = millis();
        nh.subscribe(sub);
    }

    void checkTimeout()
    {
        if (millis() - this->last_cmd_time > this->TIMEOUT_MS)
        {
            if (this->vehicle != nullptr)
            {
                this->vehicle->executeEmercencyStop();
            }
        }
    }
};
