#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "ferrari_common/msg/control_cmd.hpp"
#include "control_interface.hpp"

enum class JoyButtons
{
    TRIANGLE,
    CIRCLE,
    CROSS,
    SQUARE,
    L1,
    R1,
    L2,
    R2,
    SELECT,
    START,
    LEFT_STICK_Z,
    RIGHT_STICK_Z
};

enum class JoyAxes
{
    LEFT_STICK_X,
    LEFT_STICK_Y,
    RIGHT_STICK_X,
    RIGHT_STICK_Y
};

class JoyInterface : public ControlInterface
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);

public:
    JoyInterface();
    ~JoyInterface();
};
