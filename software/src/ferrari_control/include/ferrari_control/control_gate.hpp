#pragma once

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

/**
 * @brief ControlGateNode is a ROS2 node that manages the control commands for a vehicle, allowing
 * switching between teleoperation and autonomous modes. It subscribes to teleoperation and autonomous command topics,
 * publishes the appropriate command to the vehicle, and provides a service to toggle between modes.
 */
class ControlGateNode : public rclcpp::Node
{
public:
    /**
     * @brief Enum representing the gate mode of the control node.
     */
    enum class GateMode : uint8_t
    {
        TELEOP = 0,
        AUTO = 1
    };

    /**
     * @brief Constructor for the ControlGateNode.
     * @param options Node options for initialization.
     */
    explicit ControlGateNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    /**
     * @brief Callback function for handling teleoperation commands.
     * @param msg The incoming teleoperation command message.
     */
    void teleopCmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

    /**
     * @brief Callback function for handling autonomous commands.
     * @param msg The incoming autonomous command message.
     */
    void autoCmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

    /**
     * @brief Main control loop for managing command publication.
     */
    void controlLoop();

    /**
     * @brief Service callback for switching gate modes.
     * @param request The incoming service request.
     * @param response The outgoing service response.
     */
    void handleSwitchGateMode(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    ackermann_msgs::msg::AckermannDriveStamped makeStopCommand() const;

    /**
     * @brief Check if a command is fresh based on its timestamp and timeout.
     * @param stamp The timestamp of the command.
     * @param timeout The maximum allowed age of the command.
     * @return True if the command is fresh, false otherwise.
     */
    bool isCommandFresh(const rclcpp::Time &stamp, const rclcpp::Duration &timeout) const;

    // ROS2 communication members
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr teleop_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr auto_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr switch_gate_mode_srv_;

    // Last received commands and their timestamps
    ackermann_msgs::msg::AckermannDriveStamped last_teleop_cmd_;
    ackermann_msgs::msg::AckermannDriveStamped last_auto_cmd_;
    rclcpp::Time last_teleop_time_;
    rclcpp::Time last_auto_time_;

    // Timer and timeout configurations
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Duration teleop_timeout_{0, 0};
    rclcpp::Duration auto_timeout_{0, 0};

    // Node parameters
    GateMode gate_mode_;
    double control_rate_hz_;
    double teleop_timeout_s_;
    double auto_timeout_s_;
};
