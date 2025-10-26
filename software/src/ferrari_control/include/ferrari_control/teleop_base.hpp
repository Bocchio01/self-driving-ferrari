#pragma once

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <string>

class TeleopBase : public rclcpp::Node
{
public:
    explicit TeleopBase(
        const std::string &node_name,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

protected:
    void publishAckermannCmd(double normalized_speed, double normalized_steering);
    void publishStopCmd();
    void requestArmToggle();
    void requestModeToggle();

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr teleop_cmd_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gate_mode_client_;

    double max_steering_angle_rad_;
    double max_speed_mps_;
    std::string output_frame_id_;
};