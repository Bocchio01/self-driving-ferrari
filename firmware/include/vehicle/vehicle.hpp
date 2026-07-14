#pragma once

#include <rcl/rcl.h>
#include "vehicle/kinematics/kinematic.hpp"

namespace vehicle::interfaces
{
    class Vehicle
    {
    public:
        virtual ~Vehicle() = default;

        virtual void setMotionCommand(const void *motion_cmd) = 0;
        virtual void update() = 0;
        virtual vehicle::interfaces::OdometryState getOdometryState() = 0;
        virtual void executeEmergencyStop() = 0;
        virtual bool arm() = 0;
        virtual bool disarm() = 0;
        virtual bool isArmed() = 0;
    };
}

namespace vehicle
{
    template <class TKinematic>
    class Vehicle : public interfaces::Vehicle
    {
    public:
        virtual ~Vehicle() = default;

        void bindKinematics(TKinematic *kinematic_ptr) { this->kinematics_ = kinematic_ptr; }

        void setMotionCommand(const void *motion_cmd) override { this->kinematics_->setMotionCommand(motion_cmd); }
        void update() override { this->kinematics_->update(); }
        vehicle::interfaces::OdometryState getOdometryState() override { return this->kinematics_->getOdometryState(); }
        void executeEmergencyStop() override { this->kinematics_->executeEmergencyStop(); }
        bool arm() override { return this->kinematics_->arm(); }
        bool disarm() override { return this->kinematics_->disarm(); }
        bool isArmed() override { return this->kinematics_->isArmed(); }

    private:
        TKinematic *kinematics_ = nullptr;
    };
}