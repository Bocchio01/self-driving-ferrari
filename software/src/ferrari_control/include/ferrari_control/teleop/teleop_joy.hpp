#pragma once

#include "ferrari_control/teleop/teleop_base.hpp"

#include <sensor_msgs/msg/joy.hpp>

/**
 * @brief Node for teleoperation control using a joystick.
 * This class extends the TeleopBase class and provides functionality to read joystick inputs
 * and publish teleoperation commands based on the joystick axes and button states.
 */
class TeleopJoyNode : public TeleopBase
{
public:
    explicit TeleopJoyNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    /**
     * @brief Callback function for processing joystick input messages.
     * @param msg The joystick message containing axes and button states.
     */
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    /**
     * @brief Reads the value of a specific axis from the joystick message.
     * @param msg The joystick message.
     * @param axis_index The index of the axis to read.
     * @return The value of the specified axis, or 0.0 if the index is out of bounds.
     * @note The axis value is expected to be in the range [-1.0, 1.0].
     */
    double readAxis(const sensor_msgs::msg::Joy &msg, int axis_index) const;

    /**
     * @brief Reads the state of a specific button from the joystick message.
     * @param msg The joystick message.
     * @param button_index The index of the button to read.
     * @return True if the button is pressed, false otherwise. Returns false if the index is out of bounds.
     * @note The button state is expected to be either 0 (not pressed) or 1 (pressed).
     */
    bool readButton(const sensor_msgs::msg::Joy &msg, int button_index) const;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Mapping of joystick axes and buttons
    int steering_axis_;
    int speed_axis_;
    int toggle_arm_actuators_button_;
    int switch_gate_mode_button_;

    // Node parameters
    bool invert_steering_;
    bool invert_speed_;

    // State variables to track button presses
    bool toggle_arm_actuators_prev_;
    bool switch_gate_mode_prev_;
};
