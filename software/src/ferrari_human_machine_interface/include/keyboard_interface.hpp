#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "ferrari_common/msg/control_cmd.hpp"
#include "control_interface.hpp"

enum class ArrowKeys
{
    UP = 65,
    DOWN,
    RIGHT,
    LEFT
};

class KeyboardInterface : public ControlInterface
{
private:
    int16_t steering;
    int16_t throttle;
    bool horn;

    void reset();

public:
    KeyboardInterface();
    ~KeyboardInterface();

    void run();
};
