#pragma once

#include "ferrari_control/controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class LongitudinalPidControllerNode : public Controller
{
public:
    explicit LongitudinalPidControllerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &state) override;

    void computeControlCommand() override;

private:
    double kp_{0.0};
    double ki_{0.0};
    double kd_{0.0};

    double max_acceleration_{0.0};
    double max_braking_{0.0};

    double integral_error_{0.0};
    double previous_error_{0.0};
    rclcpp::Time last_time_;
};
