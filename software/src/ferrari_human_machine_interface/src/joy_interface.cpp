#include "joy_interface.hpp"

JoyInterface::JoyInterface()
{
    this->sub_joy = nh.subscribe("/joy", 1, &JoyInterface::joyCallback, this);
}

JoyInterface::~JoyInterface()
{
}

void JoyInterface::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
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

    this->publishControlCmd(-1.0 * joy->axes[static_cast<int>(JoyAxes::LEFT_STICK_X)],
                            +1.0 * joy->axes[static_cast<int>(JoyAxes::RIGHT_STICK_Y)],
                            -1.0 * joy->axes[static_cast<int>(JoyAxes::RIGHT_STICK_X)],
                            joy->buttons[static_cast<int>(JoyButtons::TRIANGLE)]);

    usleep(10000);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_interface");
    JoyInterface joy_interface;
    ros::spin();
}
