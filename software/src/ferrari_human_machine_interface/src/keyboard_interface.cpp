#include <ros/ros.h>
#include "keyboard_interface.hpp"
#include "KBHit.hpp"

KeyboardInterface::KeyboardInterface()
{
}

KeyboardInterface::~KeyboardInterface()
{
}

void KeyboardInterface::run()
{
    KBHit kb;

    while (ros::ok())
    {
        this->reset();

        if (kb.hit())
        {
            switch (getchar())
            {
            case 'm':
                this->handleSetGateMode();
                break;
            case 'a':
                this->handleToggleArmVehicle();
                break;
            case static_cast<int>(ArrowKeys::UP):
                this->throttle = 1.0;
                break;
            case static_cast<int>(ArrowKeys::DOWN):
                this->throttle = -1.0;
                break;
            case static_cast<int>(ArrowKeys::LEFT):
                this->steering = -1.0;
                break;
            case static_cast<int>(ArrowKeys::RIGHT):
                this->steering = 1.0;
                break;
            case ' ':
                this->horn = true;
                break;
            default:
                this->reset();
                break;
            }

            this->publishControlCmd(this->steering, this->throttle, this->steering, this->horn);
        }

        ros::spinOnce();
        usleep(10000);
    }
}

void KeyboardInterface::reset()
{
    this->steering = 0.0;
    this->throttle = 0.0;
    this->horn = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_interface");
    KeyboardInterface keyboard_interface;
    keyboard_interface.run();

    ros::spin();
}
