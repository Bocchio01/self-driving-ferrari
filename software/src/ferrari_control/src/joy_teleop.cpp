#include "ferrari_control/joy_teleop.hpp"

#include <algorithm>
#include <functional>

using std::placeholders::_1;

JoyTeleopNode::JoyTeleopNode(const rclcpp::NodeOptions &options)
    : TeleopBase("joy_teleop_node", options),
      steering_axis_(0),
      speed_axis_(1),
      deadman_button_(4),
      arm_toggle_button_(7),
      mode_toggle_button_(6),
      invert_steering_(false),
      invert_speed_(false),
      deadman_was_pressed_(false),
      arm_button_prev_(false),
      mode_button_prev_(false)
{
    steering_axis_ = this->declare_parameter<int>("steering_axis", 0);
    speed_axis_ = this->declare_parameter<int>("speed_axis", 1);
    deadman_button_ = this->declare_parameter<int>("deadman_button", 4);
    arm_toggle_button_ = this->declare_parameter<int>("arm_toggle_button", 7);
    mode_toggle_button_ = this->declare_parameter<int>("mode_toggle_button", 6);
    invert_steering_ = this->declare_parameter<bool>("invert_steering", false);
    invert_speed_ = this->declare_parameter<bool>("invert_speed", false);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 20, std::bind(&JoyTeleopNode::joyCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "joy_teleop_node started");
}

void JoyTeleopNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    handleButtonEdges(*msg);

    const bool deadman_pressed = readButton(*msg, deadman_button_);
    if (!deadman_pressed)
    {
        if (deadman_was_pressed_)
        {
            publishStopCmd();
            RCLCPP_DEBUG(this->get_logger(), "Dead-man released: sent stop command");
        }

        deadman_was_pressed_ = false;
        return;
    }

    deadman_was_pressed_ = true;

    const double steer_sign = invert_steering_ ? -1.0 : 1.0;
    const double speed_sign = invert_speed_ ? -1.0 : 1.0;
    const double normalized_steering = steer_sign * readAxis(*msg, steering_axis_);
    const double normalized_speed = speed_sign * readAxis(*msg, speed_axis_);

    publishAckermannCmd(normalized_speed, normalized_steering);
}

void JoyTeleopNode::handleButtonEdges(const sensor_msgs::msg::Joy &msg)
{
    const bool arm_pressed = readButton(msg, arm_toggle_button_);
    if (arm_pressed && !arm_button_prev_)
    {
        requestArmToggle();
    }
    arm_button_prev_ = arm_pressed;

    const bool mode_pressed = readButton(msg, mode_toggle_button_);
    if (mode_pressed && !mode_button_prev_)
    {
        requestModeToggle();
    }
    mode_button_prev_ = mode_pressed;
}

double JoyTeleopNode::readAxis(const sensor_msgs::msg::Joy &msg, int axis_index) const
{
    if (axis_index < 0 || static_cast<size_t>(axis_index) >= msg.axes.size())
    {
        return 0.0;
    }

    return std::clamp(static_cast<double>(msg.axes[static_cast<size_t>(axis_index)]), -1.0, 1.0);
}

bool JoyTeleopNode::readButton(const sensor_msgs::msg::Joy &msg, int button_index) const
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
    auto node = std::make_shared<JoyTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
