#pragma once

#include "ferrari_control/controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class LateralPurePursuitControllerNode : public Controller
{
public:
    explicit LateralPurePursuitControllerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    rcl_interfaces::msg::SetParametersResult onParamsChanged(const std::vector<rclcpp::Parameter> &parameters) override;

    // Assuming a virtual method in the Controller base class to be called periodically
    void computeControlCommand() override;

private:
    double lookahead_distance_{0.0};
    double min_lookahead_{0.0};
    double lookahead_gain_{0.0};
};