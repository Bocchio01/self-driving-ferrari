#pragma once

#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ferrari_control/controller.hpp"

class JoypadControllerNode : public Controller
{
public:
    explicit JoypadControllerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void computeControlCommand() override;
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    double readAxis(const sensor_msgs::msg::Joy &msg, int axis_index) const;
    bool readButton(const sensor_msgs::msg::Joy &msg, int button_index) const;
    void requestToggleActuators() const;
    void requestSwitchController() const;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr toggle_actuators_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr switch_controller_client_;

    int steering_axis_;
    int speed_axis_;
    int toggle_actuators_button_;
    int switch_controller_button_;
    bool invert_steering_;
    bool invert_speed_;
    bool toggle_actuators_prev_;
    bool switch_controller_prev_;
};