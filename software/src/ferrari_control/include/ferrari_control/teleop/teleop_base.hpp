#pragma once

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <string>

/**
 * @brief Base class for teleoperation control.
 *
 * This class provides functionality to publish teleoperation commands and
 * request services for engaging the vehicle and switching control gate modes.
 */
class TeleopBase : public rclcpp::Node
{
public:
    explicit TeleopBase(
        const std::string &node_name,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

protected:
    /**
     * @brief Publishes a teleoperation command to the /teleop_cmd topic.
     * @param normalized_speed The normalized speed value in the range [-1.0, 1.0].
     * @param normalized_steering The normalized steering value in the range [-1.0, 1.0].
     */
    void publishTeleopCmd(double normalized_speed, double normalized_steering);

    /**
     * @brief Requests to toggle the vehicle engagement.
     */
    void requestToggleEngageVehicle();

    /**
     * @brief Requests to switch the gate mode.
     */
    void requestSwitchGateMode();

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr teleop_cmd_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr engage_vehicle_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gate_mode_client_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr kinematic_limits_client_;
    rclcpp::TimerBase::SharedPtr kinematic_limits_retry_timer_;

    double max_speed_ = 1.0;
    double max_steering_angle_ = 1.0;

private:
    void fetchKinematicLimits();
    double extractDoubleFromJson(const std::string &json, const std::string &key, double fallback);
};