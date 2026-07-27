#pragma once

#include "ferrari_vehicle/vehicle_model.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

namespace vehicle_models::ackermann::kinematic_frenet
{

    /**
     * @brief Struct representing the state of the vehicle.
     */
    struct State
    {
        double s{0.0};  // arc length along the reference path
        double d{0.0};  // signed lateral error from the reference path
        double mu{0.0}; // heading error relative to the path tangent
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
     * @brief Class representing the kinematic bicycle model in Frenet coordinates.
     *
     * State:   [s, d, mu]
     * Control: [speed, steering_angle]
     *
     * Continuous dynamics (kinematic bicycle model in Frenet coordinates):
     *   sdot  = v * cos(mu) / (1 - d * kappa(s))
     *   ddot  = v * sin(mu)
     *   mudot = v / L * tan(delta) - kappa(s) * sdot
     *
     * Path curvature kappa(s) can be injected via setCurvatureFunction() and
     * its derivative via setCurvatureDerivativeFunction(). If the derivative is not
     * supplied, it defaults to zero, which is the usual piecewise-locally-constant-curvature assumption.
     */
    class Model : public VehicleModel<Model, 3, 2>
    {
    public:
        using Base = VehicleModel<Model, 3, 2>;
        using StateVector = Base::StateVector;
        using ControlVector = Base::ControlVector;
        using StateJacobian = Base::StateJacobian;
        using ControlJacobian = Base::ControlJacobian;

        using CurvatureFn = std::function<double(double)>;

        explicit Model(const Params &params)
            : params_(params),
              kappa_fn_([](double)
                        { return 0.0; }),
              kappa_prime_fn_([](double)
                              { return 0.0; }) {}

        // ---- curvature injection ----
        void setCurvatureFunction(CurvatureFn kappa_fn) { kappa_fn_ = std::move(kappa_fn); }
        void setCurvatureDerivativeFunction(CurvatureFn kappa_prime_fn)
        {
            kappa_prime_fn_ = std::move(kappa_prime_fn);
        }
        double kappa(double s) const { return kappa_fn_(s); }

        static StateVector toVector(const State &s)
        {
            StateVector v;
            v << s.s, s.d, s.mu;
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
            const double s = state(0);
            const double d = state(1);
            const double mu = state(2);
            const double v = u(0);
            const double delta = u(1);

            const double k = kappa_fn_(s);
            const double k_prime = kappa_prime_fn_(s);
            const double cos_mu = std::cos(mu);
            const double sin_mu = std::sin(mu);
            const double cos_delta = std::cos(delta);
            const double sec2_delta = 1.0 / (cos_delta * cos_delta);

            const double g = 1.0 - d * k; // denominator, 1 - d*kappa(s)
            const double g2 = g * g;
            const double s_dot = v * cos_mu / g; // nonlinear value at the operating point

            // Partial derivatives of sdot = v*cos(mu) / (1 - d*kappa(s)).
            const double d_sdot_ds = v * cos_mu * (d * k_prime) / g2;
            const double d_sdot_dd = v * cos_mu * k / g2;
            const double d_sdot_dmu = -v * sin_mu / g;

            StateJacobian Ac = StateJacobian::Zero();
            // Row 0: sdot
            Ac(0, 0) = d_sdot_ds;
            Ac(0, 1) = d_sdot_dd;
            Ac(0, 2) = d_sdot_dmu;
            // Row 1: ddot = v*sin(mu)  -> only depends on mu.
            Ac(1, 2) = v * cos_mu;
            // Row 2: mudot = v/L*tan(delta) - kappa(s)*sdot
            Ac(2, 0) = -k_prime * s_dot - k * d_sdot_ds;
            Ac(2, 1) = -k * d_sdot_dd;
            Ac(2, 2) = -k * d_sdot_dmu;

            ControlJacobian Bc = ControlJacobian::Zero();
            Bc(0, 0) = cos_mu / g;                                           // d(sdot)/dv
            Bc(1, 0) = sin_mu;                                               // d(ddot)/dv
            Bc(2, 0) = std::tan(delta) / params_.wheelbase - k * cos_mu / g; // d(mudot)/dv
            Bc(2, 1) = v * sec2_delta / params_.wheelbase;                   // d(mudot)/ddelta

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
            const double s = x(0);
            const double d = x(1);
            const double mu = x(2);
            const double v = u(0);
            const double delta = u(1);

            const double k = kappa_fn_(s);
            const double g = 1.0 - d * k;
            const double s_dot = v * std::cos(mu) / g;

            StateVector xdot;
            xdot(0) = s_dot;
            xdot(1) = v * std::sin(mu);
            xdot(2) = v * std::tan(delta) / params_.wheelbase - k * s_dot;
            return xdot;
        }

        Params params_;
        CurvatureFn kappa_fn_;
        CurvatureFn kappa_prime_fn_;
    };

}
