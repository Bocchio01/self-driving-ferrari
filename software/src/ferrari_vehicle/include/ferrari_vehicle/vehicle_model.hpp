#pragma once

#include <Eigen/Dense>

namespace vehicle_models
{

    /**
     * @brief Base class for vehicle models using the Curiously Recurring Template Pattern (CRTP).
     *
     * This class provides a common interface for vehicle models, allowing for static polymorphism and compile-time optimizations.
     * Derived classes must implement the required methods for state propagation, linearization, and control sanitization.
     *
     * @tparam Derived The derived class implementing the specific vehicle model
     * @tparam StateDim The dimension of the state vector
     * @tparam ControlDim The dimension of the control vector
     */
    template <typename Derived, int StateDim, int ControlDim>
    class VehicleModel
    {
    public:
        static constexpr int kStateDim = StateDim;
        static constexpr int kControlDim = ControlDim;

        using StateVector = Eigen::Matrix<double, StateDim, 1>;              // x
        using ControlVector = Eigen::Matrix<double, ControlDim, 1>;          // u
        using StateJacobian = Eigen::Matrix<double, StateDim, StateDim>;     // A
        using ControlJacobian = Eigen::Matrix<double, StateDim, ControlDim>; // B

        /**
         * @brief Propagates the vehicle state forward in time using the model's dynamics.
         *
         * This function takes the current state of the vehicle, the control input, and a time step,
         * and returns the predicted next state after applying the control input for the specified duration.
         * The control input is sanitized to ensure it respects the vehicle's physical limits
         * before being applied to the dynamics.
         *
         * Integration is performed using a fixed-step Runge-Kutta 4th order (RK4) method, which provides a
         * good balance between accuracy and computational efficiency for real-time applications.
         *
         * @param state The current state vector of the vehicle
         * @param control The applied control input vector
         * @param dt The time step for propagation
         * @return StateVector The predicted next state vector after applying the control input for dt seconds
         */
        StateVector step(const StateVector &state, const ControlVector &control, double dt) const
        {
            return derived().stepImpl(state, control, dt);
        }

        /**
         * @brief Computes the linearized dynamics of the vehicle model around a given state and control input.
         *
         * This function calculates the Jacobian matrices A and B, which represent the linearized dynamics of
         * the vehicle model around the specified state and control input.
         *
         * @param state The current state vector of the vehicle
         * @param control The applied control input vector
         * @param dt The time step for discretization
         * @param A Output parameter for the state Jacobian matrix
         * @param B Output parameter for the control Jacobian matrix
         *
         * @note The Jacobians are computed based on the continuous-time dynamics and then discretized
         * using a first-order approximation via first-order Taylor expansion.
         */
        void getLinearized(const StateVector &state, const ControlVector &control, double dt, StateJacobian &A, ControlJacobian &B) const
        {
            derived().getLinearizedImpl(state, control, dt, A, B);
        }

        /**
         * @brief Sanitizes the control input to ensure it respects the vehicle's physical limits.
         *
         * @param control The raw control input vector
         * @return ControlVector The sanitized control input vector, clamped to the vehicle's limits
         */
        ControlVector sanitizeControl(const ControlVector &control) const
        {
            return derived().sanitizeControlImpl(control);
        }

    protected:
        /**
         * @brief Accessor for the derived class instance, enabling CRTP functionality.
         *
         * This function allows the base class to call methods implemented in the derived class without using virtual functions.
         * It casts the current instance to the derived class type, enabling static polymorphism.
         *
         * @return const Derived& A reference to the derived class instance
         */
        const Derived &derived() const { return static_cast<const Derived &>(*this); }

        /**
         * @brief Non-const accessor for the derived class instance.
         *
         * @return Derived& A reference to the derived class instance
         */
        Derived &derived() { return static_cast<Derived &>(*this); }
    };

    /**
     * @brief Performs a single integration step using the 4th-order Runge-Kutta (RK4) method.
     *
     * This function integrates the continuous-time dynamics of a system over a specified time step using the RK4 method.
     * The RK4 method provides a good balance between accuracy and computational efficiency, making it suitable
     * for real-time applications such as vehicle control and simulation.
     *
     * @tparam StateVector The type of the state vector (fixed-size Eigen type)
     * @tparam ControlVector The type of the control vector (fixed-size Eigen type)
     * @tparam Dynamics A callable type that computes the continuous-time state derivative given the current state and control input
     * @param f The dynamics function that computes the state derivative
     * @param x The current state vector
     * @param u The current control input vector
     * @param dt The time step for integration
     * @return StateVector The predicted next state vector after applying the control input for dt seconds
     */
    template <typename StateVector, typename ControlVector, typename Dynamics>
    inline StateVector rk4Step(Dynamics &&f, const StateVector &x, const ControlVector &u, double dt)
    {
        const StateVector k1 = f(x, u);
        const StateVector k2 = f(StateVector(x + 0.5 * dt * k1), u);
        const StateVector k3 = f(StateVector(x + 0.5 * dt * k2), u);
        const StateVector k4 = f(StateVector(x + dt * k3), u);
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }

} // namespace vehicle_models
