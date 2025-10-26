#include "joy_interface.hpp"

JoyInterface::JoyInterface()
    : ControlInterface("joy_interface")
{
    this->sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyInterface::joyCallback, this, std::placeholders::_1));
}

JoyInterface::~JoyInterface()
{
}

void JoyInterface::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    if (joy->buttons[static_cast<int>(JoyButtons::START)])
    {
        this->handleToggleArmVehicle();
        return;
    }

    if (joy->buttons[static_cast<int>(JoyButtons::SELECT)])
    {
        this->handleSetGateMode();
        return;
    }

    this->publishControlCmd(
        static_cast<int16_t>(+100 * joy->axes[static_cast<int>(JoyAxes::LEFT_STICK_X)]),
        static_cast<int16_t>(+100 * joy->axes[static_cast<int>(JoyAxes::RIGHT_STICK_Y)]),
        static_cast<int16_t>(-100 * joy->axes[static_cast<int>(JoyAxes::RIGHT_STICK_X)]),
        joy->buttons[static_cast<int>(JoyButtons::TRIANGLE)]);

    // Sleep to allow other callbacks to process
    rclcpp::Rate(10).sleep();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto joy_interface = std::make_shared<JoyInterface>();
    rclcpp::spin(joy_interface);
    rclcpp::shutdown();
    return 0;
}
