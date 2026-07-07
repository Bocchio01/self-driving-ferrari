#include "ferrari_control/teleop/teleop_base.hpp"

#include <algorithm>
#include <chrono>

TeleopBase::TeleopBase(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options)
{
    teleop_cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("teleop_cmd", 10);
    engage_vehicle_client_ = this->create_client<std_srvs::srv::Trigger>("toggle_engage_vehicle");
    gate_mode_client_ = this->create_client<std_srvs::srv::Trigger>("switch_gate_mode");
}

void TeleopBase::publishTeleopCmd(double normalized_speed, double normalized_steering)
{
    ackermann_msgs::msg::AckermannDriveStamped cmd;

    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.drive.speed = std::clamp(normalized_speed, -1.0, 1.0);
    cmd.drive.steering_angle = std::clamp(normalized_steering, -1.0, 1.0);

    teleop_cmd_pub_->publish(cmd);
}

void TeleopBase::requestToggleEngageVehicle()
{
    if (!engage_vehicle_client_->wait_for_service(std::chrono::milliseconds(10)))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Service /toggle_engage_vehicle not available");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    engage_vehicle_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            try
            {
                auto response = future.get();
                if (response->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Successfully toggled vehicle engagement");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to toggle vehicle engagement: %s", response->message.c_str());
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Service call threw an exception: %s", e.what());
            }
        });
}

void TeleopBase::requestSwitchGateMode()
{
    if (!gate_mode_client_->wait_for_service(std::chrono::milliseconds(10)))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Service /switch_gate_mode not available");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    gate_mode_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            try
            {
                auto response = future.get();
                if (response->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Successfully switched gate mode");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to switch gate mode: %s", response->message.c_str());
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Service call threw an exception: %s", e.what());
            }
        });
}