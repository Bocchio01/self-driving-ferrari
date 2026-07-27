#pragma once

#include "ferrari_vehicle/vehicle_model.hpp"

#include <algorithm>
#include <cmath>

namespace vehicle_models::ackermann::dynamic_cartesian
{

    /**
     * @brief Struct representing the state of the vehicle.
     */
    struct State
    {
        double x{0.0};        // global x position [m]
        double y{0.0};        // global y position [m]
        double yaw{0.0};      // global heading angle [rad]
        double vx{0.0};       // longitudinal velocity in body frame [m/s] TODO: check if this is actually in body frame or global frame
        double vy{0.0};       // lateral velocity in body frame [m/s] TODO: check if this is actually in body frame or global frame
        double yaw_rate{0.0}; // yaw rate (angular velocity about vertical axis
    };

    /**
     * @brief Struct representing the command to the vehicle.
     */
    struct Command
    {
        double acceleration{0.0};   // longitudinal acceleration command [m/s^2]
        double steering_angle{0.0}; // steering angle command [rad]
    };

    /**
     * @brief Struct representing the parameters of the vehicle model.
     */
    struct Params
    {
        double mass{3.0};
        double wheelbase{0.26};
        double distance_cg_front{0.13};         // lf: CG to front axle
        double distance_cg_rear{0.13};          // lr: CG to rear axle
        double cornering_stiffness_front{50.0}; // Cf
        double cornering_stiffness_rear{50.0};  // Cr
        double max_accel{4.0};
        double max_steer{0.5};
        // Yaw (vertical-axis) moment of inertia. Not part of the originally
        // requested parameter list, but the yaw-rate equation of motion is
        // undefined without it -- there is no physically meaningful dynamic
        // bicycle model that omits it. Defaulted to a reasonable value for a
        // small ground vehicle; tune this to match the real platform.
        double yaw_inertia{0.05};
    };

    /**
     * @brief Class representing the dynamic bicycle model in Cartesian coordinates.
     *
     * State:   [x, y, yaw, vx, vy, yaw_rate]
     * Control: [acceleration, steering_angle]
     *
     * Continuous dynamics (dynamic bicycle model, rear-axle reference):
     *   xdot   = vx*cos(yaw) - vy*sin(yaw)
     *   ydot   = vx*sin(yaw) + vy*cos(yaw)
     *   yawdot = r
     *   vxdot  = a + vy*r - Fyf*sin(delta) / m
     *   vydot  = (Fyf*cos(delta) + Fyr) / m - vx*r
     *   rdot   = (lf*Fyf*cos(delta) - lr*Fyr) / Iz
     *
     * The model assumes a linear tire model (small-slip-angle approximation) and
     * includes the effects of lateral and longitudinal dynamics, making it suitable for
     * simulating more aggressive maneuvers compared to kinematic models.
     * In particular, we assume:
     *   alpha_f = delta - (vy + lf*r) / vx        front slip angle
     *   alpha_r = -(vy - lr*r) / vx                rear slip angle
     *   Fyf = Cf * alpha_f
     *   Fyr = Cr * alpha_r
     *
     * To avoid singularities in the slip-angle calculations, the longitudinal speed vx
     * is clamped to a minimum value (kMinLongitudinalSpeed) when it approaches zero.
     */
    class Model : public VehicleModel<Model, 6, 2>
    {
    public:
        using Base = VehicleModel<Model, 6, 2>;
        using StateVector = Base::StateVector;
        using ControlVector = Base::ControlVector;
        using StateJacobian = Base::StateJacobian;
        using ControlJacobian = Base::ControlJacobian;

        explicit Model(const Params &params) : params_(params) {}

        static StateVector toVector(const State &s)
        {
            StateVector v;
            v << s.x, s.y, s.yaw, s.vx, s.vy, s.yaw_rate;
            return v;
        }

        static ControlVector toVector(const Command &c)
        {
            ControlVector v;
            v << c.acceleration, c.steering_angle;
            return v;
        }

        static State toStruct(const StateVector &v)
        {
            return State{v(0), v(1), v(2), v(3), v(4), v(5)};
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
            const double vy = state(4);
            const double r = state(5);
            const double vx = clampMinLongitudinalSpeed(state(3));

            const double a_cmd = u(0);
            (void)a_cmd; // acceleration only enters vxdot linearly via B, kept for clarity
            const double delta = u(1);

            const double m = params_.mass;
            const double lf = params_.distance_cg_front;
            const double lr = params_.distance_cg_rear;
            const double Cf = params_.cornering_stiffness_front;
            const double Cr = params_.cornering_stiffness_rear;
            const double Iz = params_.yaw_inertia;

            const double cos_yaw = std::cos(yaw);
            const double sin_yaw = std::sin(yaw);
            const double cos_delta = std::cos(delta);
            const double sin_delta = std::sin(delta);

            const double alpha_f = delta - (vy + lf * r) / vx;
            const double Fyf = Cf * alpha_f;
            // Note: Fyr's value is not needed here, only its partials (below) --
            // it only enters the Jacobian linearly through d_Fyr_d(*).

            const double inv_vx2 = 1.0 / (vx * vx);

            // Slip-angle partials
            const double d_af_dvx = (vy + lf * r) * inv_vx2;
            const double d_af_dvy = -1.0 / vx;
            const double d_af_dr = -lf / vx;
            const double d_af_ddelta = 1.0;

            const double d_ar_dvx = (vy - lr * r) * inv_vx2;
            const double d_ar_dvy = -1.0 / vx;
            const double d_ar_dr = lr / vx;

            // Tire-force partials (linear in slip angle)
            const double d_Fyf_dvx = Cf * d_af_dvx;
            const double d_Fyf_dvy = Cf * d_af_dvy;
            const double d_Fyf_dr = Cf * d_af_dr;
            const double d_Fyf_ddelta = Cf * d_af_ddelta;

            const double d_Fyr_dvx = Cr * d_ar_dvx;
            const double d_Fyr_dvy = Cr * d_ar_dvy;
            const double d_Fyr_dr = Cr * d_ar_dr;
            // Fyr does not depend on delta.

            StateJacobian Ac = StateJacobian::Zero();

            // Row 0: xdot = vx*cos(yaw) - vy*sin(yaw)
            Ac(0, 2) = -vx * sin_yaw - vy * cos_yaw;
            Ac(0, 3) = cos_yaw;
            Ac(0, 4) = -sin_yaw;

            // Row 1: ydot = vx*sin(yaw) + vy*cos(yaw)
            Ac(1, 2) = vx * cos_yaw - vy * sin_yaw;
            Ac(1, 3) = sin_yaw;
            Ac(1, 4) = cos_yaw;

            // Row 2: yawdot = r
            Ac(2, 5) = 1.0;

            // Row 3: vxdot = a + vy*r - Fyf*sin(delta)/m
            Ac(3, 3) = -sin_delta / m * d_Fyf_dvx;
            Ac(3, 4) = r - sin_delta / m * d_Fyf_dvy;
            Ac(3, 5) = vy - sin_delta / m * d_Fyf_dr;

            // Row 4: vydot = (Fyf*cos(delta) + Fyr)/m - vx*r
            Ac(4, 3) = (cos_delta * d_Fyf_dvx + d_Fyr_dvx) / m - r;
            Ac(4, 4) = (cos_delta * d_Fyf_dvy + d_Fyr_dvy) / m;
            Ac(4, 5) = (cos_delta * d_Fyf_dr + d_Fyr_dr) / m - vx;

            // Row 5: rdot = (lf*Fyf*cos(delta) - lr*Fyr)/Iz
            Ac(5, 3) = (lf * cos_delta * d_Fyf_dvx - lr * d_Fyr_dvx) / Iz;
            Ac(5, 4) = (lf * cos_delta * d_Fyf_dvy - lr * d_Fyr_dvy) / Iz;
            Ac(5, 5) = (lf * cos_delta * d_Fyf_dr - lr * d_Fyr_dr) / Iz;

            ControlJacobian Bc = ControlJacobian::Zero();
            // vxdot
            Bc(3, 0) = 1.0;                                               // d(vxdot)/da
            Bc(3, 1) = -(Fyf * cos_delta + sin_delta * d_Fyf_ddelta) / m; // d(vxdot)/ddelta
            // vydot
            Bc(4, 0) = 0.0;
            Bc(4, 1) = (cos_delta * d_Fyf_ddelta - Fyf * sin_delta) / m; // d(vydot)/ddelta
            // rdot
            Bc(5, 0) = 0.0;
            Bc(5, 1) = lf * (cos_delta * d_Fyf_ddelta - Fyf * sin_delta) / Iz; // d(rdot)/ddelta

            A = StateJacobian::Identity() + Ac * dt;
            B = Bc * dt;
        }

        ControlVector sanitizeControlImpl(const ControlVector &control) const
        {
            ControlVector u = control;
            u(0) = std::clamp(u(0), -params_.max_accel, params_.max_accel);
            u(1) = std::clamp(u(1), -params_.max_steer, params_.max_steer);
            return u;
        }

    private:
        static constexpr double kMinLongitudinalSpeed = 0.1; // [m/s], singularity guard

        static double clampMinLongitudinalSpeed(double vx)
        {
            if (std::abs(vx) < kMinLongitudinalSpeed)
            {
                return std::copysign(kMinLongitudinalSpeed, vx == 0.0 ? 1.0 : vx);
            }
            return vx;
        }

        StateVector continuousDynamics(const StateVector &x, const ControlVector &u) const
        {
            const double yaw = x(2);
            const double vy = x(4);
            const double r = x(5);
            const double vx = clampMinLongitudinalSpeed(x(3));

            const double a = u(0);
            const double delta = u(1);

            const double m = params_.mass;
            const double lf = params_.distance_cg_front;
            const double lr = params_.distance_cg_rear;
            const double Cf = params_.cornering_stiffness_front;
            const double Cr = params_.cornering_stiffness_rear;
            const double Iz = params_.yaw_inertia;

            const double alpha_f = delta - (vy + lf * r) / vx;
            const double alpha_r = -(vy - lr * r) / vx;
            const double Fyf = Cf * alpha_f;
            const double Fyr = Cr * alpha_r;

            const double cos_yaw = std::cos(yaw);
            const double sin_yaw = std::sin(yaw);
            const double cos_delta = std::cos(delta);
            const double sin_delta = std::sin(delta);

            StateVector xdot;
            xdot(0) = vx * cos_yaw - vy * sin_yaw;
            xdot(1) = vx * sin_yaw + vy * cos_yaw;
            xdot(2) = r;
            xdot(3) = a + vy * r - Fyf * sin_delta / m;
            xdot(4) = (Fyf * cos_delta + Fyr) / m - vx * r;
            xdot(5) = (lf * Fyf * cos_delta - lr * Fyr) / Iz;
            return xdot;
        }

        Params params_;
    };

}
