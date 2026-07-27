#pragma once

#include "ferrari_vehicle/vehicle_model.hpp"

#include <algorithm>
#include <cmath>

namespace vehicle_models::ackermann::kinematic_cartesian
{

    /**
     * @brief Struct representing the state of the vehicle.
     */
    struct State
    {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    /**
     * @brief Struct representing the command to the vehicle.
     */
    struct Command
    {
        double speed{0.0};
        double steering_angle{0.0};
    };

    /**
     * @brief Struct representing the parameters of the vehicle model.
     */
    struct Params
    {
        double wheelbase{0.26};
        double max_speed{3.0};
        double max_steer{0.5};
    };

    /**
     * @brief Class representing the kinematic bicycle model in Cartesian coordinates.
     *
     * State:   [x, y, yaw]
     * Control: [speed, steering_angle]
     *
     * Continuous dynamics (kinematic bicycle model, rear-axle reference):
     *   xdot   = v * cos(yaw)
     *   ydot   = v * sin(yaw)
     *   yawdot = v / L * tan(delta)
     */
    class Model : public VehicleModel<Model, 3, 2>
    {
    public:
        using Base = VehicleModel<Model, 3, 2>;
        using StateVector = Base::StateVector;
        using ControlVector = Base::ControlVector;
        using StateJacobian = Base::StateJacobian;
        using ControlJacobian = Base::ControlJacobian;

        explicit Model(const Params &params) : params_(params) {}

        static StateVector toVector(const State &s)
        {
            StateVector v;
            v << s.x, s.y, s.yaw;
            return v;
        }

        static ControlVector toVector(const Command &c)
        {
            ControlVector v;
            v << c.speed, c.steering_angle;
            return v;
        }

        static State toStruct(const StateVector &v)
        {
            return State{v(0), v(1), v(2)};
        }

        static Command ToStruct(const ControlVector &v)
        {
            return Command{v(0), v(1)};
        }

        const Params &params() const { return params_; }
        Params &params() { return params_; }

        StateVector stepImpl(const StateVector &state, const ControlVector &control, double dt) const
        {
            const ControlVector u = sanitizeControlImpl(control);
            auto f = [this](const StateVector &x, const ControlVector &u_in)
            {
                return continuousDynamics(x, u_in);
            };
            return rk4Step<StateVector, ControlVector>(f, state, u, dt);
        }

        void getLinearizedImpl(const StateVector &state, const ControlVector &control, double dt, StateJacobian &A, ControlJacobian &B) const
        {
            const ControlVector u = sanitizeControlImpl(control);
            const double yaw = state(2);
            const double v = u(0);
            const double delta = u(1);
            const double cos_yaw = std::cos(yaw);
            const double sin_yaw = std::sin(yaw);
            const double tan_delta = std::tan(delta);
            const double cos_delta = std::cos(delta);
            const double sec2_delta = 1.0 / (cos_delta * cos_delta);

            StateJacobian Ac = StateJacobian::Zero();
            Ac(0, 2) = -v * sin_yaw;
            Ac(1, 2) = v * cos_yaw;

            ControlJacobian Bc = ControlJacobian::Zero();
            Bc(0, 0) = cos_yaw;
            Bc(1, 0) = sin_yaw;
            Bc(2, 0) = tan_delta / params_.wheelbase;
            Bc(2, 1) = v * sec2_delta / params_.wheelbase;

            A = StateJacobian::Identity() + Ac * dt;
            B = Bc * dt;
        }

        ControlVector sanitizeControlImpl(const ControlVector &control) const
        {
            ControlVector u = control;
            u(0) = std::clamp(u(0), -params_.max_speed, params_.max_speed);
            u(1) = std::clamp(u(1), -params_.max_steer, params_.max_steer);
            return u;
        }

    private:
        StateVector continuousDynamics(const StateVector &x, const ControlVector &u) const
        {
            const double yaw = x(2);
            const double v = u(0);
            const double delta = u(1);

            StateVector xdot;
            xdot(0) = v * std::cos(yaw);
            xdot(1) = v * std::sin(yaw);
            xdot(2) = v * std::tan(delta) / params_.wheelbase;
            return xdot;
        }

        Params params_;
    };

}
