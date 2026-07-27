#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <std_msgs/msg/float64.hpp>

#include "ferrari_control/controller.hpp"
#include "ferrari_control/solvers/mpc_solver.hpp"
#include "ferrari_vehicle/ackermann_models/dynamic_cartesian.hpp"

// Dynamic (force/slip-aware) MPC. Optimizes acceleration + steering jointly
// over the nonlinear-then-linearized bicycle model with tire forces, using
// the same linear-MPC solver as the kinematic frenet controller.
//
// The friction coefficient is exposed both as a parameter and as a live
// subscription so a road-condition / friction estimator node can update it
// online. It is used in two ways:
//   1. It caps the reference speed at each point of the local path via the
//      standard cornering-speed bound v_max = sqrt(mu * g / kappa).
//   2. It caps the actuator's acceleration bound (mu * g), refreshed every
//      control cycle, so the optimizer is never allowed to request more
//      longitudinal force than the current surface can support.
class MpcControllerNode : public Controller
{
public:
    explicit MpcControllerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

protected:
    void computeControlCommand() override;
    LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

    vehicle_models::ackermann::dynamic_cartesian::State computeCartesianState(const nav_msgs::msg::Odometry &odom) const;

    std::vector<Eigen::VectorXd> buildReferenceTrajectory(
        const vehicle_models::ackermann::dynamic_cartesian::State &current_state,
        const ferrari_planning::msg::Trajectory &trajectory) const;

private:
    rcl_interfaces::msg::SetParametersResult onParamsChanged(const std::vector<rclcpp::Parameter> &parameters) override;

    std::unique_ptr<MpcSolver> mpc_solver_;
    vehicle_models::ackermann::dynamic_cartesian::Model model_;
    vehicle_models::ackermann::dynamic_cartesian::Command last_cmd_;

    int Np_, Nc_;
    double friction_coefficient_;
    double gravity_;
};
