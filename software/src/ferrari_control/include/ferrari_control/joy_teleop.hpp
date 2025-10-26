#pragma once

#include "ferrari_control/teleop_base.hpp"

#include <sensor_msgs/msg/joy.hpp>

class JoyTeleopNode : public TeleopBase
{
public:
    explicit JoyTeleopNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void handleButtonEdges(const sensor_msgs::msg::Joy &msg);
    double readAxis(const sensor_msgs::msg::Joy &msg, int axis_index) const;
    bool readButton(const sensor_msgs::msg::Joy &msg, int button_index) const;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    int steering_axis_;
    int speed_axis_;
    int deadman_button_;
    int arm_toggle_button_;
    int mode_toggle_button_;

    bool invert_steering_;
    bool invert_speed_;

    bool deadman_was_pressed_;
    bool arm_button_prev_;
    bool mode_button_prev_;
};
