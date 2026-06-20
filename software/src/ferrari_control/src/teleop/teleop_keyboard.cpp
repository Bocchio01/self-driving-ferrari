#include "ferrari_control/teleop/teleop_keyboard.hpp"

#include <cstdio>
#include <functional>

using namespace std::chrono_literals;

TeleopKeyboardNode::TeleopKeyboardNode(const rclcpp::NodeOptions &options)
    : TeleopBase("teleop_keyboard_node", options),
      kb_()
{
    timer_ = this->create_wall_timer(50ms, std::bind(&TeleopKeyboardNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "teleop_keyboard_node started");
}

void TeleopKeyboardNode::timerCallback()
{
    double normalized_speed = 0.0;
    double normalized_steering = 0.0;

    while (kb_.hit() > 0)
    {
        const KeyCode key_code = readKeyCode();
        switch (key_code)
        {
        case KeyCode::Up:
            normalized_speed = +1.0;
            break;
        case KeyCode::Down:
            normalized_speed = -1.0;
            break;
        case KeyCode::Left:
            normalized_steering = -1.0;
            break;
        case KeyCode::Right:
            normalized_steering = +1.0;
            break;
        case KeyCode::ToggleEngageVehicle:
            requestToggleEngageVehicle();
            break;
        case KeyCode::SwitchGateMode:
            requestSwitchGateMode();
            break;
        case KeyCode::Unknown:
        default:
            break;
        }
    }

    publishTeleopCmd(normalized_speed, normalized_steering);
}

TeleopKeyboardNode::KeyCode TeleopKeyboardNode::readKeyCode()
{
    const int first_byte = std::getchar();
    if (first_byte == EOF)
    {
        return KeyCode::Unknown;
    }

    switch (first_byte)
    {
    case 'e':
        return KeyCode::ToggleEngageVehicle;
    case 'm':
        return KeyCode::SwitchGateMode;

    // WASD keys for movement
    case 'w':
        return KeyCode::Up;
    case 's':
        return KeyCode::Down;
    case 'a':
        return KeyCode::Left;
    case 'd':
        return KeyCode::Right;

    // Arrow keys are represented by a sequence of three bytes: 27 (ESC), 91 ([),
    // and a final byte indicating the direction.
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
    auto node = std::make_shared<TeleopKeyboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}