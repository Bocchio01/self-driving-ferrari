#pragma once

#include "ferrari_control/teleop_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include "KBHit.hpp"

class KeyboardTeleopNode : public TeleopBase
{
public:
    explicit KeyboardTeleopNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    enum class KeyCode
    {
        Unknown,
        Up,
        Down,
        Left,
        Right,
        Space,
        ArmToggle,
        ModeToggle,
    };

    void timerCallback();
    KeyCode readKeyCode();

    KBHit kb_;
    rclcpp::TimerBase::SharedPtr timer_;
};