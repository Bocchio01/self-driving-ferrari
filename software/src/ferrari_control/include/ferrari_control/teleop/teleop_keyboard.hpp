#pragma once

#include "ferrari_control/teleop/teleop_base.hpp"
#include "ferrari_control/teleop/KBHit.hpp"

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Node for teleoperation control using a keyboard.
 * This class extends the TeleopBase class and provides functionality to read keyboard inputs
 * and publish teleoperation commands based on the key presses.
 */
class TeleopKeyboardNode : public TeleopBase
{
public:
    explicit TeleopKeyboardNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    enum class KeyCode
    {
        Unknown,
        Up,
        Down,
        Left,
        Right,
        ToggleEngageVehicle,
        SwitchGateMode,
    };

    /**
     * @brief Timer callback function.
     */
    void timerCallback();

    /**
     * @brief Reads the key code from the keyboard.
     * @return The key code.
     */
    KeyCode readKeyCode();

    KBHit kb_;
    rclcpp::TimerBase::SharedPtr timer_;
};