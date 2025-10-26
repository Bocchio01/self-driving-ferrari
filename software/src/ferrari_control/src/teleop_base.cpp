#include "ferrari_control/teleop_base.hpp"

#include <algorithm>
#include <chrono>

TeleopBase::TeleopBase(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options),
      max_steering_angle_rad_(0.0),
      max_speed_mps_(0.0),
      output_frame_id_("base_link")
{
    max_steering_angle_rad_ = this->declare_parameter<double>("max_steering_angle_rad", 0.6);
    max_speed_mps_ = this->declare_parameter<double>("max_speed_mps", 2.0);
    output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", "base_link");

    teleop_cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/teleop_cmd", 10);
    arm_client_ = this->create_client<std_srvs::srv::Trigger>("/toggle_arm");
    gate_mode_client_ = this->create_client<std_srvs::srv::Trigger>("/toggle_gate_mode");
}

void TeleopBase::publishAckermannCmd(double normalized_speed, double normalized_steering)
{
    const double clamped_speed = std::clamp(normalized_speed, -1.0, 1.0);
    const double clamped_steering = std::clamp(normalized_steering, -1.0, 1.0);

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = output_frame_id_;
    cmd.drive.speed = clamped_speed * max_speed_mps_;
    cmd.drive.steering_angle = clamped_steering * max_steering_angle_rad_;

    teleop_cmd_pub_->publish(cmd);
}

void TeleopBase::publishStopCmd()
{
    publishAckermannCmd(0.0, 0.0);
}

void TeleopBase::requestArmToggle()
{
    if (!arm_client_->wait_for_service(std::chrono::milliseconds(1)))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Service /toggle_arm not available");
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    arm_client_->async_send_request(request);
}

void TeleopBase::requestModeToggle()
{
    if (!gate_mode_client_->wait_for_service(std::chrono::milliseconds(1)))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Service /toggle_gate_mode not available");
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    gate_mode_client_->async_send_request(request);
}