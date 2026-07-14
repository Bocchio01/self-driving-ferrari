#pragma once

namespace vehicle::interfaces
{
    struct OdometryState
    {
        float x = 0.0f;         // [m]
        float y = 0.0f;         // [m]
        float theta = 0.0f;     // [rad] (Yaw)
        float linear_x = 0.0f;  // [m/s]
        float angular_z = 0.0f; // [rad/s]
    };

    class Kinematic
    {
    public:
        enum class Type
        {
            ACKERMANN,
            DIFFERENTIAL,
            OMNIDIRECTIONAL,
            UNKNOWN
        };

        virtual ~Kinematic() = default;

        Type getType() const { return this->kinematic_type; }
        OdometryState getOdometryState() const { return this->odometry_state; }
        void setOdometryState(const OdometryState &state) { this->odometry_state = state; }

        virtual void setMotionCommand(const void *motion_cmd) = 0;
        virtual void update() = 0;
        virtual void executeEmergencyStop() = 0;
        virtual bool arm() = 0;
        virtual bool disarm() = 0;
        virtual bool isArmed() = 0;

    protected:
        Type kinematic_type;
        OdometryState odometry_state;
    };
}