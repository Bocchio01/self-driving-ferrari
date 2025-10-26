#include <rclcpp/rclcpp.hpp>
#include "keyboard_interface.hpp"
#include "KBHit.hpp"

KeyboardInterface::KeyboardInterface()
{
    this->steering = 0;
    this->throttle = 0;
    this->horn = false;
}

KeyboardInterface::~KeyboardInterface()
{
}

void KeyboardInterface::run()
{
    KBHit kb;

    while (rclcpp::ok())
    {
        this->reset();

        if (kb.hit()) // Check if a key is pressed
        {
            switch (getchar()) // Read the pressed key
            {
            case 'm':
                this->handleSetGateMode();
                break;
            case 'a':
                this->handleToggleArmVehicle();
                break;
            case static_cast<int>(ArrowKeys::UP):
                this->throttle = +100;
                break;
            case static_cast<int>(ArrowKeys::DOWN):
                this->throttle = -100;
                break;
            case static_cast<int>(ArrowKeys::LEFT):
                this->steering = -100;
                break;
            case static_cast<int>(ArrowKeys::RIGHT):
                this->steering = +100;
                break;
            case ' ':
                this->horn = true;
                break;
            default:
                this->reset();
                break;
            }

            // Publish control commands based on the inputs
            this->publishControlCmd(this->steering, this->throttle, 0, this->horn);
        }

        // Sleep to avoid high CPU usage (use rclcpp::Rate instead of usleep)
        rclcpp::Rate(10).sleep(); // Sleep for 10 Hz
    }
}

void KeyboardInterface::reset()
{
    this->steering = 0;
    this->throttle = 0;
    this->horn = false;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto keyboard_interface = std::make_shared<KeyboardInterface>();
    keyboard_interface->run();

    // Spin the node to keep the program running
    rclcpp::spin(keyboard_interface);
    rclcpp::shutdown();
    return 0;
}
