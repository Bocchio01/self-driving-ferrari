#include "ferrari_control/teleop/teleop_base.hpp"

#include <algorithm>
#include <chrono>

TeleopBase::TeleopBase(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options)
{
    teleop_cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("teleop_cmd", 10);
    engage_vehicle_client_ = this->create_client<std_srvs::srv::Trigger>("toggle_arm_actuators");
    gate_mode_client_ = this->create_client<std_srvs::srv::Trigger>("switch_gate_mode");
    kinematic_limits_client_ = this->create_client<std_srvs::srv::Trigger>("get_kinematic_limits");

    kinematic_limits_retry_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]()
        {
            if (kinematic_limits_client_->wait_for_service(std::chrono::milliseconds(0)))
            {
                kinematic_limits_retry_timer_->cancel(); // Stop checking once it's available
                fetchKinematicLimits();                  // Make the async call
            }
        });
}

void TeleopBase::fetchKinematicLimits()
{
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    kinematic_limits_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            try
            {
                auto response = future.get();
                if (response->success)
                {
                    const std::string &payload = response->message;

                    // Extract the values, using our current limits as safe fallbacks
                    this->max_speed_ = extractDoubleFromJson(payload, "max_speed", this->max_speed_);
                    this->max_steering_angle_ = extractDoubleFromJson(payload, "max_steering", this->max_steering_angle_);

                    RCLCPP_INFO(this->get_logger(),
                                "Kinematic limits updated from Teensy: max_speed=%.3f, max_steering=%.3f",
                                this->max_speed_, this->max_steering_angle_);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to get kinematic limits: %s", response->message.c_str());
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Service call threw an exception: %s", e.what());
            }
        });
}

void TeleopBase::publishTeleopCmd(double normalized_speed, double normalized_steering)
{
    ackermann_msgs::msg::AckermannDriveStamped cmd;

    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.drive.speed = std::clamp(normalized_speed, -1.0, 1.0) * max_speed_;
    cmd.drive.steering_angle = std::clamp(normalized_steering, -1.0, 1.0) * max_steering_angle_;

    RCLCPP_DEBUG(this->get_logger(),
                 "Publishing teleop command: speed=%.3f, steering_angle=%.3f",
                 cmd.drive.speed, cmd.drive.steering_angle);

    teleop_cmd_pub_->publish(cmd);
}

void TeleopBase::requestToggleEngageVehicle()
{
    if (!engage_vehicle_client_->wait_for_service(std::chrono::milliseconds(10)))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Service /toggle_arm_actuators not available");
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

double TeleopBase::extractDoubleFromJson(const std::string &json, const std::string &key, double fallback)
{
    // Find the exact key wrapped in quotes
    std::string search_target = "\"" + key + "\"";
    size_t key_pos = json.find(search_target);
    if (key_pos == std::string::npos)
        return fallback;

    // Find the colon immediately following the key
    size_t colon_pos = json.find(':', key_pos);
    if (colon_pos == std::string::npos)
        return fallback;

    // Find the start of the actual number payload
    size_t num_start = json.find_first_of("-0123456789.", colon_pos);
    if (num_start == std::string::npos)
        return fallback;

    // Convert string to double (std::stod safely stops at the first non-numeric character)
    try
    {
        return std::stod(json.substr(num_start));
    }
    catch (...)
    {
        return fallback; // Safe fallback if parsing fails
    }
}