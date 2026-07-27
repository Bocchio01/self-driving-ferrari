#pragma once

#include "ferrari_control/controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class LateralStanleyControllerNode : public Controller
{
public:
    explicit LateralStanleyControllerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    rcl_interfaces::msg::SetParametersResult onParamsChanged(const std::vector<rclcpp::Parameter> &parameters) override;

    void computeControlCommand() override;

private:
    double k_gain_{0.0};
    double k_soft_{0.0};
};