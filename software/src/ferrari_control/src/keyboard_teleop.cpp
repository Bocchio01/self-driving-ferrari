#include "ferrari_control/keyboard_teleop.hpp"

#include <cstdio>
#include <functional>

using namespace std::chrono_literals;

KeyboardTeleopNode::KeyboardTeleopNode(const rclcpp::NodeOptions &options)
    : TeleopBase("keyboard_teleop_node", options),
      kb_()
{
    timer_ = this->create_wall_timer(50ms, std::bind(&KeyboardTeleopNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "keyboard_teleop_node started");
}

void KeyboardTeleopNode::timerCallback()
{
    bool deadman_pressed = false;
    bool movement_command_active = false;
    double normalized_speed = 0.0;
    double normalized_steering = 0.0;

    while (kb_.hit() > 0)
    {
        const KeyCode key_code = readKeyCode();
        switch (key_code)
        {
        case KeyCode::Up:
            normalized_speed = +1.0;
            movement_command_active = true;
            break;
        case KeyCode::Down:
            normalized_speed = -1.0;
            movement_command_active = true;
            break;
        case KeyCode::Left:
            normalized_steering = +1.0;
            movement_command_active = true;
            break;
        case KeyCode::Right:
            normalized_steering = -1.0;
            movement_command_active = true;
            break;
        case KeyCode::Space:
            deadman_pressed = true;
            break;
        case KeyCode::ArmToggle:
            requestArmToggle();
            break;
        case KeyCode::ModeToggle:
            requestModeToggle();
            break;
        case KeyCode::Unknown:
        default:
            break;
        }
    }

    if (deadman_pressed && movement_command_active)
    {
        publishAckermannCmd(normalized_speed, normalized_steering);
        return;
    }

    publishStopCmd();
}

KeyboardTeleopNode::KeyCode KeyboardTeleopNode::readKeyCode()
{
    const int first_byte = std::getchar();
    if (first_byte == EOF)
    {
        return KeyCode::Unknown;
    }

    switch (first_byte)
    {
    case 'a':
        return KeyCode::ArmToggle;
    case 'm':
        return KeyCode::ModeToggle;
    case ' ':
        return KeyCode::Space;
    case 27:
    {
        if (kb_.hit() < 2)
        {
            return KeyCode::Unknown;
        }

        if (std::getchar() != '[')
        {
            return KeyCode::Unknown;
        }

        switch (std::getchar())
        {
        case 'A':
            return KeyCode::Up;
        case 'B':
            return KeyCode::Down;
        case 'C':
            return KeyCode::Right;
        case 'D':
            return KeyCode::Left;
        default:
            return KeyCode::Unknown;
        }
    }
    default:
        return KeyCode::Unknown;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}