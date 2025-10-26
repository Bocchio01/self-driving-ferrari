#pragma once

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class VehicleCmdGateNode : public rclcpp::Node
{
public:
    enum class GateMode : uint8_t
    {
        TELEOP = 0,
        AUTO = 1
    };

    explicit VehicleCmdGateNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void teleopCmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void autoCmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void controlLoop();
    void handleToggleGateMode(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handleToggleArm(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    ackermann_msgs::msg::AckermannDriveStamped makeStopCommand() const;
    bool isCommandFresh(const rclcpp::Time &stamp, const rclcpp::Duration &timeout) const;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr teleop_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr auto_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr toggle_gate_mode_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr toggle_arm_srv_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    ackermann_msgs::msg::AckermannDriveStamped last_teleop_cmd_;
    ackermann_msgs::msg::AckermannDriveStamped last_auto_cmd_;
    rclcpp::Time last_teleop_time_;
    rclcpp::Time last_auto_time_;

    GateMode gate_mode_;
    bool armed_;
    double control_rate_hz_;
    double teleop_timeout_s_;
    double auto_timeout_s_;
    std::string output_frame_id_;
};
