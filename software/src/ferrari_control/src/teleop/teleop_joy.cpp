#include "ferrari_control/teleop/teleop_joy.hpp"

#include <algorithm>
#include <functional>

using std::placeholders::_1;

TeleopJoyNode::TeleopJoyNode(const rclcpp::NodeOptions &options)
    : TeleopBase("teleop_joy_node", options),
      steering_axis_(0),
      speed_axis_(1),
      toggle_arm_actuators_button_(7),
      switch_gate_mode_button_(6),
      invert_steering_(false),
      invert_speed_(false),
      toggle_arm_actuators_prev_(false),
      switch_gate_mode_prev_(false)
{
    steering_axis_ = this->declare_parameter<int>("steering_axis", 0);
    speed_axis_ = this->declare_parameter<int>("speed_axis", 1);
    toggle_arm_actuators_button_ = this->declare_parameter<int>("toggle_arm_actuators_button", 7);
    switch_gate_mode_button_ = this->declare_parameter<int>("switch_gate_mode_button", 6);
    invert_steering_ = this->declare_parameter<bool>("invert_steering", false);
    invert_speed_ = this->declare_parameter<bool>("invert_speed", false);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 20, std::bind(&TeleopJoyNode::joyCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "teleop_joy_node started");
}

void TeleopJoyNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Check for engage vehicle toggle button press
    const bool toggle_arm_actuators_pressed = readButton(*msg, toggle_arm_actuators_button_);
    if (toggle_arm_actuators_pressed && !toggle_arm_actuators_prev_)
    {
        requestToggleEngageVehicle();
    }
    toggle_arm_actuators_prev_ = toggle_arm_actuators_pressed;

    // Check for gate mode switch button press
    const bool switch_gate_mode_pressed = readButton(*msg, switch_gate_mode_button_);
    if (switch_gate_mode_pressed && !switch_gate_mode_prev_)
    {
        requestSwitchGateMode();
    }
    switch_gate_mode_prev_ = switch_gate_mode_pressed;

    // Read axes for speed and steering
    const double normalized_steering = (invert_steering_ ? -1.0 : 1.0) * readAxis(*msg, steering_axis_);
    const double normalized_speed = (invert_speed_ ? -1.0 : 1.0) * readAxis(*msg, speed_axis_);
    publishTeleopCmd(normalized_speed, normalized_steering);
}

double TeleopJoyNode::readAxis(const sensor_msgs::msg::Joy &msg, int axis_index) const
{
    if (axis_index < 0 || static_cast<size_t>(axis_index) >= msg.axes.size())
    {
        return 0.0;
    }

    return std::clamp(static_cast<double>(msg.axes[static_cast<size_t>(axis_index)]), -1.0, 1.0);
}

bool TeleopJoyNode::readButton(const sensor_msgs::msg::Joy &msg, int button_index) const
{
    if (button_index < 0 || static_cast<size_t>(button_index) >= msg.buttons.size())
    {
        return false;
    }

    return msg.buttons[static_cast<size_t>(button_index)] != 0;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopJoyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
