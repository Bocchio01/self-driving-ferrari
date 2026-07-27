#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "ferrari_control/controller.hpp"
#include "ferrari_control/solvers/mpc_solver.hpp"
#include "ferrari_vehicle/ackermann_models/kinematic_frenet.hpp"

class LateralMpcControllerNode : public Controller
{
public:
    explicit LateralMpcControllerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

protected:
    void computeControlCommand() override;
    LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    rcl_interfaces::msg::SetParametersResult onParamsChanged(const std::vector<rclcpp::Parameter> &parameters) override;

private:
    std::unique_ptr<MpcSolver> mpc_solver_;
    vehicle_models::ackermann::kinematic_frenet::Model frenet_model_;
    vehicle_models::ackermann::kinematic_frenet::Command last_cmd_;

    int Np_, Nc_;
    double target_speed_;
};